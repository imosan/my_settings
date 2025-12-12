# render_frames.py
import bpy
import os

# 出力先フォルダ（絶対パス推奨）
out_dir = "/tmp/blender_renders"  # ← ここを自分の出力フォルダに書き換えてください
os.makedirs(out_dir, exist_ok=True)

scene = bpy.context.scene

# 出力フォーマットなど（必要に応じて変更）
scene.render.image_settings.file_format = 'PNG'   # PNGで保存
# 例: 解像度を変更したければ以下を編集
# scene.render.resolution_x = 1920
# scene.render.resolution_y = 1080
# scene.render.resolution_percentage = 100

# レンダーエンジンを明示するなら（例: 'CYCLES' または 'BLENDER_EEVEE'）
# scene.render.engine = 'CYCLES'

# レンダリングするフレームのリスト（0,10,20,...,220）
frames = range(0, 221, 10)

for f in frames:
    # フレームをセット
    try:
        scene.frame_set(f)
    except Exception as e:
        # Blenderのバージョンや設定によってはフレーム0が扱えない場合があるのでその旨を通知
        print(f"frame_set failed for frame {f}: {e}")
        continue

    # 出力ファイル名（ゼロ埋め）
    filename = f"frame_{f:03d}.png"
    filepath = os.path.join(out_dir, filename)

    # scene.render.filepath にセットしてから write_still=True で書き出す
    scene.render.filepath = filepath

    print(f"Rendering frame {f} -> {filepath}")
    bpy.ops.render.render(write_still=True)

print("All done.")
