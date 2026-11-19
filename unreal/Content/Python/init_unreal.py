"""Runs at every editor start (Python Editor Script Plugin): builds the materials once if missing."""
import unreal

_handle = None


def _build_once(_dt):
    global _handle
    unreal.unregister_slate_post_tick_callback(_handle)
    import reflectar_garments; reflectar_garments.import_all(only_missing=True)
    if unreal.EditorAssetLibrary.does_asset_exist("/Game/ReflectAR/M_Proxy"):
        return
    import reflectar_setup
    reflectar_setup.build()


# wait for the first editor tick: the asset registry and material editor are ready by then.
# Not in -game (standalone) runs: there is no editor asset subsystem and the lookup crashes.
if "-game" not in unreal.SystemLibrary.get_command_line().lower().split():
    _handle = unreal.register_slate_post_tick_callback(_build_once)
