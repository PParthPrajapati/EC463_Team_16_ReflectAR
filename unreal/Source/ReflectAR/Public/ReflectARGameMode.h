#pragma once

#include "CoreMinimal.h"
#include "GameFramework/GameModeBase.h"
#include "GameFramework/HUD.h"
#include "ReflectARGameMode.generated.h"

/** Spawns the mirror at startup; no pawn (the view comes from UMirrorLocalPlayer). */
UCLASS()
class REFLECTAR_API AReflectARGameMode : public AGameModeBase
{
	GENERATED_BODY()

public:
	AReflectARGameMode();
	virtual void StartPlay() override;
};

/**
 * Hosts the UI: SReflectARHud, a Slate widget painted over the game viewport (prompts, carousel,
 * gesture feedback; see SReflectARHud.h). Only the latency-test marker is still drawn on the Canvas.
 */
UCLASS()
class REFLECTAR_API AReflectARHUD : public AHUD
{
	GENERATED_BODY()

public:
	virtual void DrawHUD() override;
	virtual void EndPlay(const EEndPlayReason::Type Reason) override;

private:
	TSharedPtr<class SReflectARHud> Widget;
	TWeakObjectPtr<class UGameViewportClient> Viewport;
	void Panel(float X, float Y, float W, float H, const FLinearColor& C);
};
