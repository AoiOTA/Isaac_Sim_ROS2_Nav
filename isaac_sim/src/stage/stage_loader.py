"""USD layer helpers. Isaac/USD imports are intentionally function-local."""

from __future__ import annotations

from pathlib import Path
import time


class StageLoadError(RuntimeError):
    pass


def create_or_open_project_stage(path: str | Path):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    if not path.exists():
        # Seed a file-backed layer first, then attach it through Kit below.
        # Returning this standalone pxr Stage would create a split brain where
        # local validation passes but RTX/PhysX remain on Kit's anonymous Stage.
        from pxr import Usd

        seeded = Usd.Stage.CreateNew(str(path))
        if seeded is None or not seeded.GetRootLayer().Save():
            raise StageLoadError(f"failed to seed project stage {path}")
    stage = None
    if path.exists():
        try:
            import omni.usd

            context = omni.usd.get_context()
            import omni.kit.app

            def wait_for_stable_context_stage():
                deadline = time.monotonic() + 15.0
                stable = None
                stable_updates = 0
                while time.monotonic() < deadline:
                    candidate = context.get_stage()
                    candidate_path = ""
                    if candidate is not None:
                        layer = candidate.GetRootLayer()
                        candidate_path = layer.realPath or layer.identifier
                    matching = (
                        candidate_path
                        and not candidate_path.startswith("anon:")
                        and Path(candidate_path).resolve() == path
                        and context.get_stage_loading_status()[2] == 0
                    )
                    if matching:
                        if candidate is stable:
                            stable_updates += 1
                        else:
                            stable = candidate
                            stable_updates = 1
                        if stable_updates >= 5:
                            return candidate
                    else:
                        stable = None
                        stable_updates = 0
                    omni.kit.app.get_app().update()
                return None

            current = context.get_stage()
            current_path = ""
            if current is not None:
                layer = current.GetRootLayer()
                current_path = layer.realPath or layer.identifier
            if current_path and not current_path.startswith("anon:") and (
                Path(current_path).resolve() == path
            ):
                stage = wait_for_stable_context_stage()
            elif current is not None:
                # SimulationApp starts with an anonymous context Stage.  Isaac
                # 6 refuses to synchronously open the project stage while that
                # Stage remains attached; silently falling back to pxr.Usd
                # creates a second Stage that sensors cannot see.
                if not context.can_close_stage() or not context.close_stage():
                    raise StageLoadError(
                        "could not close the existing Kit Stage before project open"
                    )
            if stage is None and context.open_stage(str(path)):
                # Isaac Sim 6.0 may acknowledge open_stage() before the Kit
                # context switches away from its anonymous startup Stage.
                # Composing into that stale object passes local validation but
                # leaves RTX/PhysX attached to a different, uncomposed Stage.
                stage = wait_for_stable_context_stage()
            if stage is None:
                raise StageLoadError(f"Kit context failed to open project stage {path}")
        except StageLoadError:
            raise
        except ImportError:
            # Pure USD tests run without a Kit application. Runtime code must
            # use the context branch so sensors and OmniGraph share this stage.
            stage = None
    if stage is None:
        from pxr import Usd

        stage = Usd.Stage.Open(str(path)) if path.exists() else Usd.Stage.CreateNew(str(path))
    if stage is None:
        raise StageLoadError(f"failed to create/open USD stage {path}")
    stage.SetEditTarget(stage.GetRootLayer())
    return stage


def ensure_sublayer(root_layer, layer_path: str | Path) -> bool:
    """Add an absolute Sublayer once; return whether the layer changed."""

    layer_path = str(Path(layer_path).resolve())
    existing = list(root_layer.subLayerPaths)
    normalized = [str(Path(item).resolve()) if not item.startswith("anon:") else item for item in existing]
    if layer_path in normalized:
        return False
    root_layer.subLayerPaths.insert(0, layer_path)
    return True


def ensure_xform(stage, prim_path: str):
    from pxr import UsdGeom

    prim = stage.GetPrimAtPath(prim_path)
    if prim.IsValid():
        if prim.GetTypeName() not in {"", "Xform"}:
            raise StageLoadError(f"{prim_path} exists with incompatible type {prim.GetTypeName()}")
        return prim
    return UsdGeom.Xform.Define(stage, prim_path).GetPrim()


def ensure_reference(prim, asset_path: str | Path) -> bool:
    """Author a defaultPrim reference exactly once."""

    asset_path = str(Path(asset_path).resolve())
    reference_list = prim.GetMetadata("references")
    if reference_list is not None:
        authored = reference_list.GetAddedOrExplicitItems()
        root_layer = prim.GetStage().GetRootLayer()
        layer_path = root_layer.realPath or root_layer.identifier
        layer_directory = (
            Path(layer_path).resolve().parent
            if layer_path and not layer_path.startswith("anon:")
            else None
        )
        for item in authored:
            if not item.primPath.isEmpty or not item.assetPath:
                continue
            item_path = Path(item.assetPath)
            if not item_path.is_absolute():
                if layer_directory is None:
                    continue
                item_path = layer_directory / item_path
            if str(item_path.resolve()) == asset_path:
                return False
    if not prim.GetReferences().AddReference(asset_path):
        raise StageLoadError(f"failed to add reference {asset_path} to {prim.GetPath()}")
    return True


def repair_malformed_asset_paths(stage, source_directory: str | Path) -> tuple[str, ...]:
    """Overlay resolvable ``.../`` asset typos without editing the source USD.

    Some exported room assets use three dots where a relative path was
    intended. USD treats that text literally. Only paths whose corrected
    target exists are overridden, so unrelated or genuinely missing assets
    remain visible to the normal diagnostics.
    """

    from pxr import Sdf

    source_directory = Path(source_directory).expanduser().resolve()
    repaired: list[str] = []
    for prim in stage.TraverseAll():
        for attribute in prim.GetAttributes():
            value = attribute.Get()
            if not isinstance(value, Sdf.AssetPath) or not value.path.startswith(".../"):
                continue
            target = (source_directory / value.path[4:]).resolve()
            if not target.is_file():
                continue
            attribute.Set(Sdf.AssetPath(str(target)))
            repaired.append(f"{prim.GetPath()}.{attribute.GetName()}")
    return tuple(repaired)


def make_environment_meshes_double_sided(stage) -> tuple[str, ...]:
    """Make imported indoor meshes visible to RTX rays from either side.

    Room exporters commonly author every mesh as single-sided even though wall
    normals face away from the room interior.  Author the correction only in
    the project/runtime root layer; the source room USD remains untouched.
    Call this before the robot reference is added so only environment meshes
    are affected.
    """
    from pxr import UsdGeom

    repaired: list[str] = []
    for prim in stage.TraverseAll():
        if not prim.IsA(UsdGeom.Mesh):
            continue
        mesh = UsdGeom.Mesh(prim)
        attribute = mesh.GetDoubleSidedAttr()
        if bool(attribute.Get()):
            continue
        mesh.CreateDoubleSidedAttr().Set(True)
        repaired.append(str(prim.GetPath()))
    return tuple(repaired)


def save_stage(stage) -> None:
    if not stage.GetRootLayer().Save():
        raise StageLoadError(f"failed to save root layer {stage.GetRootLayer().identifier}")
