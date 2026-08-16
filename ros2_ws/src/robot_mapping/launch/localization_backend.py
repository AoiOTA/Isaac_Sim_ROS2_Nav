# Localization backend selection shared by localization.launch.py and its
# unit tests.  Kept free of ROS imports so tests can exercise the parsing
# rules without a sourced ROS environment.

IDEAL_LOCALIZATION_BACKEND = 'ideal'
AMCL_LOCALIZATION_BACKEND = 'amcl'
SLAM_TOOLBOX_LOCALIZATION_BACKEND = 'slam_toolbox'
VALID_LOCALIZATION_BACKENDS = frozenset({
    IDEAL_LOCALIZATION_BACKEND,
    AMCL_LOCALIZATION_BACKEND,
    SLAM_TOOLBOX_LOCALIZATION_BACKEND,
})


def resolve_localization_backend(localization_backend,
                                 use_posegraph_localization):
    """
    Resolve the mutually exclusive localization backend.

    A non-empty localization_backend wins.  An empty value falls back to the
    legacy use_posegraph_localization boolean (true -> slam_toolbox,
    false -> ideal) so existing callers keep their behavior.
    """
    backend = (localization_backend or '').strip().lower()
    if backend:
        if backend not in VALID_LOCALIZATION_BACKENDS:
            raise ValueError(
                'localization_backend must be one of {}; got {!r}'.format(
                    sorted(VALID_LOCALIZATION_BACKENDS), localization_backend))
        return backend
    legacy = (use_posegraph_localization or '').strip().lower()
    if legacy not in {'true', 'false'}:
        raise ValueError(
            'use_posegraph_localization must be true or false; '
            f'got {use_posegraph_localization!r}')
    if legacy == 'true':
        return SLAM_TOOLBOX_LOCALIZATION_BACKEND
    return IDEAL_LOCALIZATION_BACKEND
