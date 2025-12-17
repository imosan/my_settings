
#include "ThumbnailExporter.h"

#include "AssetRegistry/AssetRegistryModule.h"
#include "AssetRegistry/IAssetRegistry.h"
#include "UObject/SoftObjectPath.h"

#include "UnrealEd/Public/AssetThumbnail.h"
#include "UnrealEd/Public/AssetThumbnailPool.h"

#include "Rendering/SlateRenderer.h"
#include "Slate/SlateTextures.h"

#include "RHI.h"
#include "RenderResource.h"
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
        AssetData) != UE::AssetRegistry::EExists::Exists)
    {
        return false;
    }

    // Thumbnail
    TSharedRef<FAssetThumbnailPool> Pool =
        MakeShared<FAssetThumbnailPool>(1);

    FAssetThumbnail Thumbnail(AssetData, 256, 256, Pool);
    Thumbnail.RefreshThumbnail();

    // ★ 正しい型で取得
    FSlateTexture2DRHIRef* SlateTex =
        static_cast<FSlateTexture2DRHIRef*>(
            Thumbnail.GetViewportRenderTargetTexture());

    if (!SlateTex || !SlateTex->GetTypedResource())
        return false;

    FTexture2DRHIRef Tex = SlateTex->GetTypedResource();

    // Read pixels via RHI
    TArray<FColor> Pixels;
    FIntRect Rect(0, 0, 256, 256);

    FRHICommandListImmediate& RHICmd =
        FRHICommandListExecutor::GetImmediateCommandList();

    RHICmd.ReadSurfaceData(
        Tex,
        Rect,
        Pixels,
        FReadSurfaceDataFlags(RCM_UNorm)
    );

    // PNG encode
    TArray<uint8> PNG;
    FImageUtils::CompressImageArray(256, 256, Pixels, PNG);

    // ★ 第2引数は TCHAR*
    return FFileHelper::SaveArrayToFile(PNG, *OutputPNG);
}
