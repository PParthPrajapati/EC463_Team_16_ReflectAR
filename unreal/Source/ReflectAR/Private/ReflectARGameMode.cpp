#include "ReflectARGameMode.h"
#include "ReflectARMirror.h"
#include "SReflectARHud.h"
#include "Engine/Canvas.h"
#include "Engine/GameViewportClient.h"
#include "Engine/World.h"
#include "CanvasItem.h"

using namespace ReflectAR;

AReflectARGameMode::AReflectARGameMode()
{
	DefaultPawnClass = nullptr;
	HUDClass = AReflectARHUD::StaticClass();
}

void AReflectARGameMode::StartPlay()
{
	Super::StartPlay();
	if (!AReflectARMirror::Get(GetWorld())) GetWorld()->SpawnActor<AReflectARMirror>();
}

// ======================================================================== HUD
void AReflectARHUD::Panel(float X, float Y, float W, float H, const FLinearColor& C)
{
	FCanvasTileItem Tile(FVector2D(X, Y), FVector2D(W, H), C);
	Tile.BlendMode = SE_BLEND_Translucent;
	Canvas->DrawItem(Tile);
}

void AReflectARHUD::DrawHUD()
{
	Super::DrawHUD();
	const AReflectARMirror* Mo = AReflectARMirror::Get(GetWorld());
	if (Mo && Canvas && ReflectAR::FTrackingClient::Now() - Mo->CalibTime < 4.0)
		Canvas->DrawText(GEngine->GetLargeFont(), Mo->CalibText, 40.f, Canvas->SizeY - 120.f, 3.f, 3.f);
	if (Mo && Mo->bMirrorOutput) return;              // the mirror shows the garments and nothing else
	// the UI proper is a Slate widget over the viewport. Added on the first draw rather than in
	// BeginPlay: by now the viewport's overlay exists, in a standalone game and in Play-In-Editor alike.
	if (!Widget.IsValid())
	{
		if (UGameViewportClient* V = GetWorld()->GetGameViewport())
		{
			Widget = SNew(SReflectARHud, GetWorld()).Visibility(EVisibility::HitTestInvisible);
			V->AddViewportWidgetContent(Widget.ToSharedRef(), 10);
			Viewport = V;
		}
	}
	const AReflectARMirror* M = AReflectARMirror::Get(GetWorld());
	if (!M || !M->Client() || !Canvas) return;
	const float W = Canvas->SizeX, H = Canvas->SizeY;
	const float S = FMath::Sqrt((W / 1920.f) * (H / 1080.f));          // scale with the screen (Unity's CanvasScaler, match 0.5)

	// ---- latency test: a large marker on the right wrist, easy to time in slow-motion video
	if (M->bLatencyTest && M->bTracked && M->Body().Valid)
	{
		V3 Wp = M->Body().J[LM::RWrist];
		if (M->Mode == EMirrorViewMode::Mirror) Wp.Z = -Wp.Z;          // the reflected hand
		FVector2D Sp;
		if (M->BodyToScreen(Wp, FVector2D(W, H), Sp))
		{
			const float R = 45 * S;
			Panel(Sp.X - R - 6 * S, Sp.Y - R - 6 * S, 2 * R + 12 * S, 2 * R + 12 * S, FLinearColor(0, 0, 0, 1));
			FCanvasTileItem Dot(FVector2D(Sp.X - R, Sp.Y - R), FVector2D(2 * R, 2 * R), FLinearColor(1.f, 0.1f, 0.6f, 1.f));
			Canvas->DrawItem(Dot);
		}
	}
}

void AReflectARHUD::EndPlay(const EEndPlayReason::Type Reason)
{
	if (Widget.IsValid())
	{
		if (UGameViewportClient* V = Viewport.Get()) V->RemoveViewportWidgetContent(Widget.ToSharedRef());
		Widget.Reset();
	}
	Super::EndPlay(Reason);
}
