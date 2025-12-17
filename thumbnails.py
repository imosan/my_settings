import unreal
import os

output_dir = r"D:/Thumbs"
os.makedirs(output_dir, exist_ok=True)

assets = unreal.EditorAssetLibrary.list_assets("/Game/Models", recursive=True)

for asset_path in assets:
    asset = unreal.EditorAssetLibrary.load_asset(asset_path)
    if not asset:
        continue

    thumb = unreal.AssetThumbnail(asset, 256, 256)
    png_path = os.path.join(
        output_dir,
        asset.get_name() + ".png"
    )

    unreal.ThumbnailTools.export_thumbnail(
        asset,
        png_path
    )
