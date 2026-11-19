#include "GarmentAsset.h"
#include "Engine/StaticMesh.h"
#include "StaticMeshResources.h"

namespace ReflectAR
{
	bool LoadGarmentAsset(const FString& Name, FGarmentAsset& Out, float WeldTolerance)
	{
		const FString Path = FString::Printf(TEXT("%s/%s.%s"), GarmentAssetFolder(), *Name, *Name);
		const UStaticMesh* Mesh = LoadObject<UStaticMesh>(nullptr, *Path);
		if (!Mesh)
		{
			UE_LOG(LogTemp, Error, TEXT("ReflectAR: garment %s is missing. Import the garments/ FBX files with Content/Python/reflectar_garments.py."), *Path);
			return false;
		}
		const bool bOk = LoadGarmentAsset(Mesh, Out, WeldTolerance);
		Out.Name = Name;
		return bOk;
	}

	bool LoadGarmentAsset(const UStaticMesh* Mesh, FGarmentAsset& Out, float WeldTolerance)
	{
		Out = FGarmentAsset();
		if (!Mesh) return false;
		Out.Name = Mesh->GetName();
		const FStaticMeshRenderData* RD = Mesh->GetRenderData();
		if (!RD || RD->LODResources.Num() == 0)
		{
			UE_LOG(LogTemp, Error, TEXT("ReflectAR: garment %s has no render data."), *Out.Name);
			return false;
		}
		const FStaticMeshLODResources& LOD = RD->LODResources[0];
		const FPositionVertexBuffer& PosB = LOD.VertexBuffers.PositionVertexBuffer;
		const FStaticMeshVertexBuffer& VB = LOD.VertexBuffers.StaticMeshVertexBuffer;
		const FColorVertexBuffer& ColB = LOD.VertexBuffers.ColorVertexBuffer;
		const int32 Nr = int32(PosB.GetNumVertices());
		// without bAllowCPUAccess a cooked build drops these copies once they are on the GPU
		if (Nr == 0 || !PosB.GetVertexData() || !VB.GetTangentData() || !VB.GetTexCoordData() || LOD.IndexBuffer.GetNumIndices() == 0)
		{
			UE_LOG(LogTemp, Error, TEXT("ReflectAR: garment %s has no CPU-side mesh data (import it with Allow CPU Access)."), *Out.Name);
			return false;
		}
		const bool bColors = ColB.GetNumVertices() == uint32(Nr) && ColB.GetVertexData() != nullptr;
		if (!bColors)
			UE_LOG(LogTemp, Warning, TEXT("ReflectAR: garment %s has no vertex colours: nothing pinned, no bands."), *Out.Name);

		// ---- render vertices, in canonical garment space (Unreal cm -> body m)
		TArray<V3> RPos; RPos.SetNumUninitialized(Nr);
		Out.UV.SetNumUninitialized(Nr);
		TArray<FColor> RCol; RCol.Init(FColor(0, 0, 0, 255), Nr);
		for (int32 i = 0; i < Nr; i++)
		{
			RPos[i] = UEToBody(FVector(PosB.VertexPosition(i)));
			// the FBX importer stores V as 1 - V: undo it, so UV0 is the pattern's metres of fabric again
			const FVector2f T = VB.GetVertexUV(i, 0);
			Out.UV[i] = V2(T.X, 1.f - T.Y);
			if (bColors) RCol[i] = ColB.VertexColor(i);
		}

		// ---- weld by position: a hash grid of cells the size of the tolerance
		const float Tol = FMath::Max(WeldTolerance, 1e-6f), Tol2 = Tol * Tol;
		auto Cell = [Tol](const V3& p) { return FIntVector(FMath::FloorToInt(p.X / Tol), FMath::FloorToInt(p.Y / Tol), FMath::FloorToInt(p.Z / Tol)); };
		TMap<FIntVector, TArray<int32, TInlineAllocator<2>>> Grid;
		Grid.Reserve(Nr);
		Out.RenderToSim.SetNumUninitialized(Nr);
		TArray<FColor> SCol;
		for (int32 i = 0; i < Nr; i++)
		{
			const FIntVector C = Cell(RPos[i]);
			int32 Found = INDEX_NONE;
			for (int32 dz = -1; dz <= 1 && Found == INDEX_NONE; dz++)
				for (int32 dy = -1; dy <= 1 && Found == INDEX_NONE; dy++)
					for (int32 dx = -1; dx <= 1 && Found == INDEX_NONE; dx++)
						if (const auto* L = Grid.Find(C + FIntVector(dx, dy, dz)))
							for (int32 s : *L)
								if (V3::DistSquared(Out.Pos[s], RPos[i]) <= Tol2) { Found = s; break; }
			if (Found == INDEX_NONE)
			{
				Found = Out.Pos.Add(RPos[i]);
				SCol.Add(RCol[i]);
				Grid.FindOrAdd(C).Add(Found);
			}
			else
			{
				Out.WeldedAway++;
				const FColor& A = SCol[Found]; const FColor& B = RCol[i];
				const int32 D = FMath::Max3(FMath::Abs(A.R - B.R), FMath::Abs(A.G - B.G), FMath::Abs(A.B - B.B));
				Out.MaxColorMismatch = FMath::Max(Out.MaxColorMismatch, D / 255.f);
			}
			Out.RenderToSim[i] = Found;
		}
		const int32 Ns = Out.Pos.Num();
		Out.Pin.SetNumUninitialized(Ns); Out.Elastic.SetNumUninitialized(Ns); Out.Rib.SetNumUninitialized(Ns);
		for (int32 s = 0; s < Ns; s++)
		{
			// the importer stores FBX colours as 8-bit values without a gamma curve
			Out.Pin[s] = SCol[s].R / 255.f; Out.Elastic[s] = SCol[s].G / 255.f; Out.Rib[s] = SCol[s].B / 255.f;
		}

		// ---- triangles (all sections), wound outward
		const FIndexArrayView Idx = LOD.IndexBuffer.GetArrayView();
		Out.RenderTris.Reserve(Idx.Num());
		for (const FStaticMeshSection& Sec : LOD.Sections)
			for (uint32 k = 0; k < Sec.NumTriangles; k++)
			{
				const uint32 b = Sec.FirstIndex + k * 3;
				Out.RenderTris.Add(int32(Idx[b])); Out.RenderTris.Add(int32(Idx[b + 1])); Out.RenderTris.Add(int32(Idx[b + 2]));
			}
		// Unreal's front face is the other way around from Cross(b - a, c - a): decide by the
		// imported normals (which point outward) rather than assume, and wind outward.
		double Agree = 0;
		for (int32 t = 0; t < Out.RenderTris.Num(); t += 3)
		{
			const int32 a = Out.RenderTris[t], b = Out.RenderTris[t + 1], c = Out.RenderTris[t + 2];
			const V3 Fn = Cross(RPos[b] - RPos[a], RPos[c] - RPos[a]);
			const FVector4f Nz = VB.VertexTangentZ(a) + VB.VertexTangentZ(b) + VB.VertexTangentZ(c);
			Agree += Dot(Fn, UEToBody(FVector(Nz.X, Nz.Y, Nz.Z) * 100.0));
		}
		if (Agree < 0)
			for (int32 t = 0; t < Out.RenderTris.Num(); t += 3) Swap(Out.RenderTris[t + 1], Out.RenderTris[t + 2]);
		Out.Tris.SetNumUninitialized(Out.RenderTris.Num());
		int32 Degenerate = 0;
		for (int32 t = 0; t < Out.RenderTris.Num(); t += 3)
		{
			for (int32 e = 0; e < 3; e++) Out.Tris[t + e] = Out.RenderToSim[Out.RenderTris[t + e]];
			if (Out.Tris[t] == Out.Tris[t + 1] || Out.Tris[t + 1] == Out.Tris[t + 2] || Out.Tris[t] == Out.Tris[t + 2]) Degenerate++;
		}
		if (Degenerate)
			UE_LOG(LogTemp, Warning, TEXT("ReflectAR: garment %s: %d triangles collapsed by the weld (tolerance %.2f mm too large?)."), *Out.Name, Degenerate, Tol * 1000.f);

		Out.BoundsMin = V3(BIG_NUMBER); Out.BoundsMax = V3(-BIG_NUMBER);
		for (const V3& p : Out.Pos) { Out.BoundsMin = V3::Min(Out.BoundsMin, p); Out.BoundsMax = V3::Max(Out.BoundsMax, p); }
		return true;
	}
}
