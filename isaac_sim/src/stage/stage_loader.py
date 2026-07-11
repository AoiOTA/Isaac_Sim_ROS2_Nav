"""USD layer helpers. Isaac/USD imports are intentionally function-local."""

from __future__ import annotations

from pathlib import Path


class StageLoadError(RuntimeError):
    pass


def create_or_open_project_stage(path: str | Path):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = None
    if path.exists():
        try:
            import omni.usd

            context = omni.usd.get_context()
            if context.open_stage(str(path)):
                stage = context.get_stage()
        except (ImportError, RuntimeError):
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


def save_stage(stage) -> None:
    if not stage.GetRootLayer().Save():
        raise StageLoadError(f"failed to save root layer {stage.GetRootLayer().identifier}")
