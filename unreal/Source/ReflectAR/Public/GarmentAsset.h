#pragma once

#include "BodyMath.h"

class UStaticMesh;

namespace ReflectAR
{
	/**
	 * A garment authored as a mesh: the FBX files in <repo>/garments (made by
	 * garments/make_garments.py, later by an artist), imported by Content/Python/reflectar_garments.py
	 * as static meshes into /Game/ReflectAR/Garments with CPU access allowed.
	 *
	 * Canonical garment space (the contract): the garment at rest on the reference body in the rest
	 * pose of FRestSkeleton (shoulder joints at (+-0.18, 0.50, 0), arms 40 degrees below horizontal,
	 * hips at (+-0.11, 0, 0)), in body coordinates: metres, origin between the hip joints, +X the
	 * wearer's right, +Y up, +Z toward the wearer's front (the mirror).
	 *
	 * FBX convention: centimetres, Z-up, front axis -Y, right-handed (Unreal's own import axis
	 * system, so the importer applies no rotation or scale, only its Y flip into left-handed space).
	 * The file stores fbx = (100 z, -100 x, 100 y); after import a vertex is (100 z, 100 x, 100 y),
	 * i.e. BodyToUE(p), and the loader maps it back with UEToBody: p = (Y, Z, X) / 100.
	 *
	 * Vertex colours (stored 8-bit, read back as value / 255): R = pin weight (1 where the body
	 * carries the garment: yoke, back of the collar, sleeve heads; 0 where it hangs free),
	 * G = elastic band gripping the body (ribbed hem band, rib cuffs), B = ribbed knit region
	 * (collar band, hem band, cuffs), A unused.
	 *
	 * Reads LOD 0's CPU copies of the render buffers (positions, UV0, colours, indices; kept by
	 * bAllowCPUAccess), so it works the same in the editor, -game and a cooked build. The result
	 * holds no engine objects and can be handed to a worker thread.
	 */
	struct FGarmentAsset
	{
		FString Name;

		// simulation mesh: vertices welded by position (the importer splits them at UV seams)
		TArray<V3> Pos;                 // canonical garment space, metres
		TArray<int32> Tris;             // into Pos; Cross(b - a, c - a) points outward
		TArray<float> Pin, Elastic, Rib;// per simulation vertex, 0..1 (vertex colour R, G, B)

		// render mesh: split at UV seams
		TArray<int32> RenderToSim;      // render vertex -> simulation vertex
		TArray<V2> UV;                  // per render vertex: metres of fabric as in the FBX (the importer's V flip undone), grain along V
		TArray<int32> RenderTris;       // into the render vertices; triangle i matches Tris' triangle i

		V3 BoundsMin = V3::ZeroVector, BoundsMax = V3::ZeroVector;
		int32 WeldedAway = 0;           // render vertices merged into another one by the weld
		float MaxColorMismatch = 0.f;   // largest colour difference between render vertices welded together
	};

	/**
	 * Loads /Game/ReflectAR/Garments/<Name> (e.g. TEXT("heavyweight_tee")) and converts it. Game
	 * thread (it may load the asset). Vertices closer than WeldTolerance (m) become one simulation
	 * vertex. Returns false (and logs why) if the asset is missing or its CPU data isn't available.
	 * In a packaged build the folder must be cooked: it is loaded by name, not referenced, so add
	 * /Game/ReflectAR/Garments to Project Settings > Packaging > Additional Asset Directories to Cook.
	 */
	bool LoadGarmentAsset(const FString& Name, FGarmentAsset& Out, float WeldTolerance = 0.0005f);
	bool LoadGarmentAsset(const UStaticMesh* Mesh, FGarmentAsset& Out, float WeldTolerance = 0.0005f);

	/** Where the importer puts the garments. */
	inline const TCHAR* GarmentAssetFolder() { return TEXT("/Game/ReflectAR/Garments"); }
}
