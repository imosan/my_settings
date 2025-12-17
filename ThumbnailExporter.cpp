// ThumbnailExporter.cpp
#include "ThumbnailExporter.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetRegistry/IAssetRegistry.h"
#include "UObject/SoftObjectPath.h"

#include "UnrealEd/Public/ThumbnailTools.h"
#include "UnrealEd/Public/AssetThumbnail.h"
#include "UnrealEd/Public/AssetThumbnailPool.h"

#include "ImageUtils.h"
#include "Misc/FileHelper.h"

IMPLEMENT_MODULE(FThumbnailExporterModule, ThumbnailExporter)

bool FThumbnailExporterModule::ExportThumbnailPNG(
    const FString& ObjectPath,
    const FString& OutputPNG)
{
    // AssetRegistry
    FAssetRegistryModule& ARM =
        FModuleManager::LoadModuleChecked<FAssetRegistryModule>("AssetRegistry");

    FAssetData AssetData;
    if (ARM.Get().TryGetAssetByObjectPath(
            FSoftObjectPath(ObjectPath),
            AssetData
        ) != UE::AssetRegistry::EExists::Exists)
    {
        return false;
    }

    // Thumbnail pool
    TSharedRef<FAssetThumbnailPool> Pool =
        MakeShared<FAssetThumbnailPool>(1);

    // Create thumbnail
    FAssetThumbnail Thumbnail(AssetData, 256, 256, Pool);
    Thumbnail.RefreshThumbnail();

    // Read pixels
    FSlateShaderResource* Resource =
        Thumbnail.GetViewportRenderTargetTexture();
    if (!Resource)
        return false;

    TArray<FColor> Pixels;
    Pixels.SetNumUninitialized(256 * 256);

    Resource->ReadPixels(Pixels);

    // Encode PNG
    TArray<uint8> PNG;
    FImageUtils::CompressImageArray(256, 256, Pixels, PNG);

    return FFileHelper::SaveArrayToFile(PNG, *OutputPNG);
}
