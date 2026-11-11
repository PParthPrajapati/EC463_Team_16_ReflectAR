"""Minimal ASCII FBX 7.4 writer: one mesh, one material slot, normals, UV0 and vertex colours.

Axis convention written into GlobalSettings: Z-up, front axis -Y, right-handed, centimetres
(UpAxis 2, FrontAxis 1 sign -1, CoordAxis 0 sign +1, UnitScaleFactor 1). That is exactly
Unreal's import axis system, so the importer applies no scene rotation and no unit scale; its
right- to left-handed conversion then flips Y. The generator writes
    fbx = (100 z, -100 x, 100 y)          for a canonical body point (x, y, z) in metres,
so after import an Unreal vertex is (100 z, 100 x, 100 y) = ReflectAR::BodyToUE(x, y, z), and
the loader recovers the canonical point with UEToBody.
"""
import numpy as np


def to_fbx(p):
    p = np.asarray(p, float)
    return np.stack([p[:, 2], -p[:, 0], p[:, 1]], 1) * 100.0


def to_fbx_dir(n):
    n = np.asarray(n, float)
    return np.stack([n[:, 2], -n[:, 0], n[:, 1]], 1)


def _arr(name, a, fmt, indent):
    a = np.asarray(a).ravel()
    s = ",".join(fmt % v for v in a)
    return "%s%s: *%d {\n%s\ta: %s\n%s}\n" % (indent, name, len(a), indent, s, indent)


def _f(v):
    return "%.6f" % v


def write(path, name, X, F, N, UV, UVidx, C):
    """X (n,3) canonical metres; F (m,3) triangles wound so cross(b-a, c-a) points outward in
    canonical coordinates; N (n,3) vertex normals; UV (k,2) and UVidx (m,3) per corner;
    C (n,4) vertex colours (linear, 0..1)."""
    P = to_fbx(X)
    Nn = to_fbx_dir(N)
    # the axis change is a reflection: reverse the corners so the faces stay outward (CCW, RH)
    Fr = F[:, [0, 2, 1]]
    UVr = UVidx[:, [0, 2, 1]]
    pvi = Fr.copy().astype(np.int64)
    pvi[:, 2] = -pvi[:, 2] - 1
    corner_n = Nn[Fr.ravel()]
    corner_c = Fr.ravel()
    ind = "\t\t"
    out = []
    w = out.append
    w("; FBX 7.4.0 project file\n; ReflectAR garment generator (garments/make_garments.py)\n\n")
    w("FBXHeaderExtension:  {\n\tFBXHeaderVersion: 1003\n\tFBXVersion: 7400\n"
      "\tCreator: \"ReflectAR make_garments.py\"\n}\n")
    w("GlobalSettings:  {\n\tVersion: 1000\n\tProperties70:  {\n"
      "\t\tP: \"UpAxis\", \"int\", \"Integer\", \"\",2\n"
      "\t\tP: \"UpAxisSign\", \"int\", \"Integer\", \"\",1\n"
      "\t\tP: \"FrontAxis\", \"int\", \"Integer\", \"\",1\n"
      "\t\tP: \"FrontAxisSign\", \"int\", \"Integer\", \"\",-1\n"
      "\t\tP: \"CoordAxis\", \"int\", \"Integer\", \"\",0\n"
      "\t\tP: \"CoordAxisSign\", \"int\", \"Integer\", \"\",1\n"
      "\t\tP: \"OriginalUpAxis\", \"int\", \"Integer\", \"\",2\n"
      "\t\tP: \"OriginalUpAxisSign\", \"int\", \"Integer\", \"\",1\n"
      "\t\tP: \"UnitScaleFactor\", \"double\", \"Number\", \"\",1\n"
      "\t\tP: \"OriginalUnitScaleFactor\", \"double\", \"Number\", \"\",1\n"
      "\t}\n}\n\n")
    w("Documents:  {\n\tCount: 1\n\tDocument: 100, \"\", \"Scene\" {\n\t\tRootNode: 0\n\t}\n}\n\n")
    w("References:  {\n}\n\n")
    w("Definitions:  {\n\tVersion: 100\n\tCount: 4\n"
      "\tObjectType: \"GlobalSettings\" {\n\t\tCount: 1\n\t}\n"
      "\tObjectType: \"Model\" {\n\t\tCount: 1\n\t}\n"
      "\tObjectType: \"Geometry\" {\n\t\tCount: 1\n\t}\n"
      "\tObjectType: \"Material\" {\n\t\tCount: 1\n\t}\n}\n\n")
    w("Objects:  {\n")
    w("\tGeometry: 1000, \"Geometry::%s\", \"Mesh\" {\n" % name)
    w(_arr("Vertices", P, "%.5f", ind))
    w(_arr("PolygonVertexIndex", pvi, "%d", ind))
    w("\t\tGeometryVersion: 124\n")
    w("\t\tLayerElementNormal: 0 {\n\t\t\tVersion: 102\n\t\t\tName: \"\"\n"
      "\t\t\tMappingInformationType: \"ByPolygonVertex\"\n\t\t\tReferenceInformationType: \"Direct\"\n")
    w(_arr("Normals", corner_n, "%.6f", "\t\t\t"))
    w("\t\t}\n")
    w("\t\tLayerElementSmoothing: 0 {\n\t\t\tVersion: 102\n\t\t\tName: \"\"\n"
      "\t\t\tMappingInformationType: \"ByPolygon\"\n\t\t\tReferenceInformationType: \"Direct\"\n")
    w(_arr("Smoothing", np.ones(len(F), int), "%d", "\t\t\t"))
    w("\t\t}\n")
    w("\t\tLayerElementColor: 0 {\n\t\t\tVersion: 101\n\t\t\tName: \"Col\"\n"
      "\t\t\tMappingInformationType: \"ByPolygonVertex\"\n\t\t\tReferenceInformationType: \"IndexToDirect\"\n")
    w(_arr("Colors", C, "%.5f", "\t\t\t"))
    w(_arr("ColorIndex", corner_c, "%d", "\t\t\t"))
    w("\t\t}\n")
    w("\t\tLayerElementUV: 0 {\n\t\t\tVersion: 101\n\t\t\tName: \"UVMap\"\n"
      "\t\t\tMappingInformationType: \"ByPolygonVertex\"\n\t\t\tReferenceInformationType: \"IndexToDirect\"\n")
    w(_arr("UV", UV, "%.6f", "\t\t\t"))
    w(_arr("UVIndex", UVr.ravel(), "%d", "\t\t\t"))
    w("\t\t}\n")
    w("\t\tLayerElementMaterial: 0 {\n\t\t\tVersion: 101\n\t\t\tName: \"\"\n"
      "\t\t\tMappingInformationType: \"AllSame\"\n\t\t\tReferenceInformationType: \"IndexToDirect\"\n"
      "\t\t\tMaterials: *1 {\n\t\t\t\ta: 0\n\t\t\t}\n\t\t}\n")
    w("\t\tLayer: 0 {\n\t\t\tVersion: 100\n")
    for t in ("LayerElementNormal", "LayerElementSmoothing", "LayerElementColor", "LayerElementUV", "LayerElementMaterial"):
        w("\t\t\tLayerElement:  {\n\t\t\t\tType: \"%s\"\n\t\t\t\tTypedIndex: 0\n\t\t\t}\n" % t)
    w("\t\t}\n\t}\n")
    w("\tModel: 2000, \"Model::%s\", \"Mesh\" {\n\t\tVersion: 232\n\t\tProperties70:  {\n"
      "\t\t\tP: \"Lcl Translation\", \"Lcl Translation\", \"\", \"A\",0,0,0\n"
      "\t\t\tP: \"Lcl Rotation\", \"Lcl Rotation\", \"\", \"A\",0,0,0\n"
      "\t\t\tP: \"Lcl Scaling\", \"Lcl Scaling\", \"\", \"A\",1,1,1\n"
      "\t\t}\n\t\tShading: T\n\t\tCulling: \"CullingOff\"\n\t}\n" % name)
    w("\tMaterial: 3000, \"Material::M_%s\", \"\" {\n\t\tVersion: 102\n\t\tShadingModel: \"lambert\"\n"
      "\t\tMultiLayer: 0\n\t\tProperties70:  {\n"
      "\t\t\tP: \"DiffuseColor\", \"Color\", \"\", \"A\",0.8,0.8,0.8\n\t\t}\n\t}\n" % name)
    w("}\n\n")
    w("Connections:  {\n\tC: \"OO\",2000,0\n\tC: \"OO\",1000,2000\n\tC: \"OO\",3000,2000\n}\n")
    with open(path, "w", newline="\n") as fh:
        fh.write("".join(out))
