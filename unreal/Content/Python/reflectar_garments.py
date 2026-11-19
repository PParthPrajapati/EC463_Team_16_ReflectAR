"""Imports the garment meshes (<repo>/garments/*.fbx) as static meshes into /Game/ReflectAR/Garments.

Run in the editor (Tools > Execute Python Script, or the Output Log's Python console):
    import reflectar_garments; reflectar_garments.import_all()
or headless:
    UnrealEditor-Cmd <project> -run=pythonscript -script="<this file>" -unattended -nosplash
At editor start (init_unreal.py, first line of _build_once) only missing garments are imported:
    import reflectar_garments; reflectar_garments.import_all(only_missing=True)
After regenerating the FBX files, run import_all() to replace the assets.

GarmentAsset.cpp (ReflectAR::LoadGarmentAsset) reads them back. What it relies on:
  - one static mesh per file, named after the file, LOD 0 only, no Nanite, no collision;
  - Allow CPU Access, so a cooked build keeps the vertex/index data on the CPU;
  - positions untouched: the FBX is written in Unreal's own import axis system (Z-up, front -Y,
    right-handed, cm), so the importer applies no rotation or scale, only its Y flip; an imported
    vertex is BodyToUE(canonical point) and the loader inverts that with UEToBody;
  - the file's normals (smooth across UV seams) imported, tangents computed (MikkTSpace);
  - vertex colours kept as they are in the file (pin / elastic band / rib weights);
  - UV0 at full precision (it is in metres of fabric, not 0..1).
"""
import glob
import os

import unreal

DEST = "/Game/ReflectAR/Garments"

eal = unreal.EditorAssetLibrary


def garments_dir():
    """<repo>/garments, next to <repo>/unreal/ReflectAR.uproject."""
    project = unreal.Paths.convert_relative_path_to_full(unreal.Paths.project_dir())
    return os.path.normpath(os.path.join(project, os.pardir, "garments"))


def _pipeline(name):
    p = unreal.InterchangeGenericAssetsPipeline()
    p.set_editor_property("use_source_name_for_asset", False)
    p.set_editor_property("asset_name", name)
    p.set_editor_property("import_offset_translation", unreal.Vector(0, 0, 0))
    p.set_editor_property("import_offset_rotation", unreal.Rotator(0, 0, 0))
    p.set_editor_property("import_offset_uniform_scale", 1.0)

    common = p.get_editor_property("common_meshes_properties")
    common.set_editor_property("force_all_mesh_as_type", unreal.InterchangeForceMeshType.IFMT_STATIC_MESH)
    common.set_editor_property("import_lods", False)
    common.set_editor_property("bake_meshes", True)
    common.set_editor_property("vertex_color_import_option", unreal.InterchangeVertexColorImportOption.IVCIO_REPLACE)
    common.set_editor_property("recompute_normals", False)
    common.set_editor_property("recompute_tangents", True)
    common.set_editor_property("use_mikk_t_space", True)
    common.set_editor_property("remove_degenerates", False)
    common.set_editor_property("use_full_precision_u_vs", True)

    mesh = p.get_editor_property("mesh_pipeline")
    mesh.set_editor_property("import_static_meshes", True)
    mesh.set_editor_property("import_skeletal_meshes", False)
    mesh.set_editor_property("collision", False)
    mesh.set_editor_property("build_nanite", False)
    mesh.set_editor_property("generate_lightmap_u_vs", False)

    mat = p.get_editor_property("material_pipeline")
    mat.set_editor_property("import_materials", False)     # the renderer assigns M_Fabric
    mat.get_editor_property("texture_pipeline").set_editor_property("import_textures", False)
    return p


def _finish(path):
    """Settings the import pipeline doesn't cover, made explicit and saved."""
    sm = unreal.load_asset(path)
    if not isinstance(sm, unreal.StaticMesh):
        unreal.log_error("ReflectAR garments: %s did not import as a StaticMesh" % path)
        return None
    # the editor subsystem is missing in a commandlet (-run=pythonscript): the older library has
    # the same functions there
    sub = unreal.get_editor_subsystem(unreal.StaticMeshEditorSubsystem) or getattr(unreal, "EditorStaticMeshLibrary", None)
    if sub is None:
        unreal.log_error("ReflectAR garments: no static mesh editing API available")
        return None
    nanite = sm.get_editor_property("nanite_settings")
    nanite.set_editor_property("enabled", False)
    sm.set_editor_property("nanite_settings", nanite)
    sub.remove_collisions(sm)
    bs = sub.get_lod_build_settings(sm, 0)
    bs.set_editor_property("recompute_normals", False)
    bs.set_editor_property("recompute_tangents", True)
    bs.set_editor_property("use_mikk_t_space", True)
    bs.set_editor_property("remove_degenerates", False)
    bs.set_editor_property("use_full_precision_u_vs", True)
    bs.set_editor_property("generate_lightmap_u_vs", False)
    sub.set_lod_build_settings(sm, 0, bs)
    sub.set_allow_cpu_access(sm, True)
    if not sm.get_editor_property("allow_cpu_access"):      # (the commandlet fallback doesn't set it)
        sm.modify()
        sm.set_editor_property("allow_cpu_access", True)
    eal.save_loaded_asset(sm)
    return sm


def import_all(src=None, only_missing=False):
    """Imports every garments/*.fbx (replacing existing assets). only_missing: skip files that
    already have an asset (for the editor start-up hook)."""
    src = src or garments_dir()
    files = sorted(glob.glob(os.path.join(src, "*.fbx")))
    if not files:
        unreal.log_warning("ReflectAR garments: no .fbx files in %s" % src)
        return []
    tasks = []
    for f in files:
        name = os.path.splitext(os.path.basename(f))[0]
        if only_missing and eal.does_asset_exist("%s/%s" % (DEST, name)):
            continue
        stack = unreal.InterchangePipelineStackOverride()
        stack.add_pipeline(_pipeline(name))
        t = unreal.AssetImportTask()
        t.set_editor_property("filename", f)
        t.set_editor_property("destination_path", DEST)
        t.set_editor_property("destination_name", name)
        t.set_editor_property("replace_existing", True)
        t.set_editor_property("replace_existing_settings", True)
        t.set_editor_property("automated", True)
        t.set_editor_property("save", True)
        t.set_editor_property("async_", False)
        t.set_editor_property("options", stack)
        tasks.append((name, t))
    if not tasks:
        return []
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([t for _, t in tasks])
    done = []
    for name, _ in tasks:
        path = "%s/%s" % (DEST, name)
        sm = _finish(path) if eal.does_asset_exist(path) else None
        if sm is None:
            unreal.log_error("ReflectAR garments: importing %s failed" % name)
            continue
        b = sm.get_bounding_box()
        unreal.log("ReflectAR garments: %s  %d verts  %d tris  bounds (%.2f %.2f %.2f) .. (%.2f %.2f %.2f) cm  "
                   "cpu access %s  nanite %s  lods %d" % (
                       path, sm.get_num_vertices(0), sm.get_num_triangles(0), b.min.x, b.min.y, b.min.z,
                       b.max.x, b.max.y, b.max.z, sm.get_editor_property("allow_cpu_access"),
                       sm.get_editor_property("nanite_settings").get_editor_property("enabled"), sm.get_num_lods()))
        done.append(path)
    return done


if __name__ == "__main__":
    import_all()
