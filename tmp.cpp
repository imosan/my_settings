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

void UMyBlueprintFunctionLibrary::CaptureWithUI()
{
    if (GEngine && GEngine->GameViewport)
    {
        FHighResScreenshotConfig& HighResScreenshotConfig = GetHighResScreenshotConfig();
        HighResScreenshotConfig.Filename = TEXT("MyShot.png");

        GEngine->GameViewport->Viewport->TakeHighResScreenShot();
    }
}

#include "MyBlueprintFunctionLibrary.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "HighResScreenshot.h"
#include "Misc/Paths.h"
#include "Misc/FileHelper.h"

void UMyBlueprintFunctionLibrary::CaptureWithUI()
{
    // エンジンが生きていて、ビューポートが存在するか確認
    if (GEngine && GEngine->GameViewport)
    {
        // スクリーンショット設定取得
        FHighResScreenshotConfig& HighResScreenshotConfig = GetHighResScreenshotConfig();

        // Saved/Screenshots/Platform/ に保存される
        // 例: MyShot.png → WindowsNoEditor/Saved/Screenshots/Windows/MyShot.png
        HighResScreenshotConfig.Filename = TEXT("MyShot");

        // 高解像度スクリーンショット（UIを含む）
        // TakeHighResScreenShot() は UI も含めてレンダリング後の最終出力をキャプチャする
        GEngine->GameViewport->Viewport->TakeHighResScreenShot();

        UE_LOG(LogTemp, Log, TEXT("[CaptureWithUI] Screenshot Requested: %s"), *HighResScreenshotConfig.Filename);
    }
    else
    {
        UE_LOG(LogTemp, Warning, TEXT("CaptureWithUI: No GameViewport found."));
    }
}

// MyBlueprintFunctionLibrary.h
#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "ImageUtils.h"
#include "HighResScreenshot.h"
#include "MyBlueprintFunctionLibrary.generated.h"

UCLASS()
class YOURPROJECT_API UMyBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
    GENERATED_BODY()
public:
    UFUNCTION(BlueprintCallable, Category="Screenshot")
    static void CaptureWithUI(const FString& Filename);
};

// MyBlueprintFunctionLibrary.cpp
#include "MyBlueprintFunctionLibrary.h"
#include "Engine/Engine.h"
#include "Engine/GameViewportClient.h"
#include "Misc/FileHelper.h"
#include "HAL/PlatformFilemanager.h"

void UMyBlueprintFunctionLibrary::CaptureWithUI(const FString& Filename)
{
    if (GEngine && GEngine->GameViewport)
    {
        // ビューポート取得
        FViewport* Viewport = GEngine->GameViewport->Viewport;
        if (!Viewport)
            return;

        // ピクセルバッファ作成
        TArray<FColor> Bitmap;
        Bitmap.SetNumZeroed(Viewport->GetSizeXY().X * Viewport->GetSizeXY().Y);

        // UI込みでスクリーンを読み込む
        if (Viewport->ReadPixels(Bitmap))
        {
            // 画像反転
            for (int32 y = 0; y < Viewport->GetSizeXY().Y / 2; ++y)
            {
                for (int32 x = 0; x < Viewport->GetSizeXY().X; ++x)
                {
                    Bitmap.SwapMemory(x + y * Viewport->GetSizeXY().X, x + (Viewport->GetSizeXY().Y - 1 - y) * Viewport->GetSizeXY().X);
                }
            }

            // PNG に保存
            FIntPoint Size(Viewport->GetSizeXY());
            FHighResScreenshotConfig& Config = GetHighResScreenshotConfig();
            Config.Filename = Filename;
            FFileHelper::CreateBitmap(*Filename, Size.X, Size.Y, Bitmap.GetData());
        }
    }
}
