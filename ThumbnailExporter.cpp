#include "ThumbnailExporter.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "UnrealEd/Public/AssetThumbnail.h"
#include "UnrealEd/Public/AssetThumbnailPool.h"
#include "ImageUtils.h"
#include "Misc/FileHelper.h"

void FThumbnailExporterModule::StartupModule() {}
void FThumbnailExporterModule::ShutdownModule() {}

bool FThumbnailExporterModule::ExportThumbnail(
    const FString& AssetPath,
    const FString& OutPNGPath)
{
    FAssetRegistryModule& ARM =
        FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

    FAssetData AssetData = ARM.Get().GetAssetByObjectPath(*AssetPath);
    if (!AssetData.IsValid())
        return false;

    TSharedPtr<FAssetThumbnailPool> Pool =
        MakeShared<FAssetThumbnailPool>(1);

    FAssetThumbnail Thumbnail(AssetData, 256, 256, Pool);
    Thumbnail.RefreshThumbnail();

    FSlateShaderResource* Resource =
        Thumbnail.GetViewportRenderTargetTexture();
    if (!Resource)
        return false;

    TArray<FColor> Pixels;
    Pixels.SetNum(256 * 256);

    Resource->ReadPixels(Pixels);

    TArray<uint8> PNG;
    FImageUtils::CompressImageArray(256, 256, Pixels, PNG);

    return FFileHelper::SaveArrayToFile(PNG, *OutPNGPath);
}

IMPLEMENT_MODULE(FThumbnailExporterModule, ThumbnailExporter)
