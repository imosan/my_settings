// ScreenshotHelper.h
#pragma once
#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "ScreenshotHelper.generated.h"

UCLASS()
class UScreenshotHelper : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    // Blueprint から呼べるスクリーンショット要求
    UFUNCTION(BlueprintCallable, Category="Screenshot")
    static void RequestScreenshotWithUI(const FString& BaseFileName = TEXT("MyScreenshot"));
};

// ScreenshotHelper.cpp
#include "ScreenshotHelper.h"
#include "Misc/FileHelper.h"
#include "Misc/Paths.h"
#include "Misc/DateTime.h"
#include "HAL/FileManager.h"
#include "ImageUtils.h"
#include "IImageWrapper.h"
#include "IImageWrapperModule.h"
#include "Modules/ModuleManager.h"
#include "Engine/GameViewportClient.h"
#include "HighResScreenshot.h"
#include "Misc/ScreenshotRequest.h"

void UScreenshotHelper::RequestScreenshotWithUI(const FString& BaseFileName)
{
    // 保存先ディレクトリ (Saved/Screenshots/<ProjectName>/)
    const FString ProjectName = FApp::GetProjectName();
    const FString SaveDir = FPaths::ProjectSavedDir() / TEXT("Screenshots") / ProjectName;
    if (!IFileManager::Get().DirectoryExists(*SaveDir))
    {
        // 再帰的にディレクトリを作成
        IFileManager::Get().MakeDirectory(*SaveDir, /*Tree=*/ true);
    }

    // ファイル名にタイムスタンプを付ける（重複防止）
    const FString Timestamp = FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S"));
    const FString FileName = BaseFileName + TEXT("_") + Timestamp + TEXT(".png");
    const FString FullPath = SaveDir / FileName;

    // デリゲート（完了通知）を登録しておく
    // UGameViewportClient::OnScreenshotCaptured に Bind（ピクセル受取版）
    UGameViewportClient::OnScreenshotCaptured().AddStatic(
        [](int32 SizeX, int32 SizeY, const TArray<FColor>& Bitmap)
        {
            // このラムダはスクショ完了時に実行される（受け取ったBitmapをPNGで保存する例）
            FString SaveDirLocal = FPaths::ProjectSavedDir() / TEXT("Screenshots") / FApp::GetProjectName();
            const FString LocalFile = SaveDirLocal / FString::Printf(TEXT("captured_%s.png"), *FDateTime::Now().ToString(TEXT("%Y%m%d_%H%M%S")));

            TArray<uint8> CompressedPNG;
            FImageUtils::CompressImageArray(SizeX, SizeY, Bitmap, CompressedPNG);

            if (FFileHelper::SaveArrayToFile(CompressedPNG, *LocalFile))
            {
                UE_LOG(LogTemp, Log, TEXT("Screenshot saved (via delegate) : %s"), *LocalFile);
            }
            else
            {
                UE_LOG(LogTemp, Error, TEXT("Failed to save screenshot : %s"), *LocalFile);
            }
        }
    );

    // 実際にスクリーンショットを要求する（UI を含める）
    // 第2引数 bShowUI=true（UMG/Slate を含める）、第3引数 bAddFilenameSuffix=false（既に一意な名前なら false）
    FScreenshotRequest::RequestScreenshot(FullPath, /*bShowUI=*/ true, /*bAddFilenameSuffix=*/ false);

    UE_LOG(LogTemp, Log, TEXT("Requested screenshot: %s (including UI)"), *FullPath);
}
