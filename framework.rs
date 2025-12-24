use std::sync::Arc;
use std::cell:: RefCell;
use block_padding::UnpadError;
use wasm_bindgen::prelude::*;
use gloo_net::http::Request;
use serde::Deserialize;
use aes::cipher::{block_padding::Pkcs7, BlockDecryptMut, BlockEncryptMut, KeyIvInit};
use aes::Aes256;
use cbc::{Encryptor, Decryptor};
use futures_signals::signal::Mutable;
//use x25519_dalek::{EphemeralSecret, PublicKey};
//use rand::rngs::OsRng;
/*
use aes_gcm::{
    aead::{Aead, KeyInit, OsRng as AesOsRng},
    Aes256Gcm, Nonce
};
*/
use base64::{engine::general_purpose, Engine as _};
//use gloo_timers::future::sleep;
//use std::time::Duration;

type Aes256CbcDec = Decryptor<Aes256>;

use wgpu::{Instance, Surface, util::DeviceExt, AstcBlock, AstcChannel};
use winit::{
    dpi::PhysicalSize,
    event::{Event, KeyEvent, StartCause, WindowEvent},
    event_loop::{EventLoop, EventLoopWindowTarget, EventLoopBuilder, EventLoopProxy},
    keyboard::{Key, NamedKey},
    window::Window,
};

use crate::utils::get_adapter_with_capabilities_or_from_env;

// Pass
pub struct Pass {
    pub pipeline: wgpu::RenderPipeline,
    pub pipeline_layout: wgpu::PipelineLayout,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
    pub uniform_buf: Option<wgpu::Buffer>,
}

pub struct ShadowPass {
    pub pipeline: wgpu::RenderPipeline,
    pub pipeline_layout: wgpu::PipelineLayout,
    pub bind_group_layout: wgpu::BindGroupLayout,
}

pub struct RenderTexturePass {
    pub pipeline: wgpu::RenderPipeline,
    pub pipeline_layout: wgpu::PipelineLayout,
    pub sampler: wgpu::Sampler,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
    pub uniform_buf: Option<wgpu::Buffer>,
}

pub struct BloomComputePass {
    pub pipeline: wgpu::ComputePipeline,
    pub sampler: wgpu::Sampler,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
}

pub struct KawaseBlurComputePass {
    pub pipeline_down: wgpu::ComputePipeline,
    pub pipeline_up: wgpu::ComputePipeline,
    pub bind_group_layout: wgpu::BindGroupLayout,
    pub bind_group: wgpu::BindGroup,
    pub sampler: wgpu::Sampler,
    pub param_buffer: wgpu::Buffer,
}

// Initialize logging in platform dependant ways.
fn init_logger() {
    cfg_if::cfg_if! {
        if #[cfg(target_arch = "wasm32")] {
            // As we don't have an environment to pull logging level from, we use the query string.
            let query_string = web_sys::window().unwrap().location().search().unwrap();
            let query_level: Option<log::LevelFilter> = parse_url_query_string(&query_string, "RUST_LOG")
                .and_then(|x| x.parse().ok());

            // We keep wgpu at Error level, as it's very noisy.
            let base_level = if cfg!(debug_assertions) {
                query_level.unwrap_or(log::LevelFilter::Info)
            } else {
                log::LevelFilter::Off
            };

            let wgpu_level = if cfg!(debug_assertions) {
                query_level.unwrap_or(log::LevelFilter::Error)
            } else {
                log::LevelFilter::Off
            };

            // On web, we use fern, as console_log doesn't have filtering on a per-module level.
            fern::Dispatch::new()
                .level(base_level)
                .level_for("wgpu_core", wgpu_level)
                .level_for("wgpu_hal", wgpu_level)
                .level_for("naga", wgpu_level)
                .chain(fern::Output::call(console_log::log))
                .apply()
                .unwrap();
            std::panic::set_hook(Box::new(console_error_panic_hook::hook));
        } else {
            // parse_default_env will read the RUST_LOG environment variable and apply it on top
            // of these default filters.
            env_logger::builder()
                .filter_level(log::LevelFilter::Info)
                // We keep wgpu at Error level, as it's very noisy.
                .filter_module("wgpu_core", log::LevelFilter::Info)
                .filter_module("wgpu_hal", log::LevelFilter::Error)
                .filter_module("naga", log::LevelFilter::Error)
                .parse_default_env()
                .init();
        }
    }
}

pub trait AppMod: 'static + Sized { // Sized はデフォルト(書かなくてもSized)。わざわざ書いているのは dyn Example のトレイトオブジェクト化を禁止している
    //const SRGB: bool = true;

    fn optional_features() -> wgpu::Features {
        wgpu::Features::empty()
    }

    fn required_features() -> wgpu::Features {
        wgpu::Features::empty()
    }

    fn required_downlevel_capabilities() -> wgpu::DownlevelCapabilities {
        wgpu::DownlevelCapabilities {
            flags: wgpu::DownlevelFlags::empty(),
            shader_model: wgpu::ShaderModel::Sm5,
            ..wgpu::DownlevelCapabilities::default()
        }
    }

    fn required_limits() -> wgpu::Limits {
        wgpu::Limits::downlevel_webgl2_defaults() // These downlevel limits will allow the code to run on all possible hardware
    }

    fn init(
        config: &wgpu::SurfaceConfiguration,
        adapter: &wgpu::Adapter,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        sky_texture: wgpu::Texture,
    ) -> Self;

    fn resize(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        new_surface_size: PhysicalSize<u32>,
    );

    fn get_aspect_ratio(&self) -> f32 {
        1.0
    }

    fn update(&mut self, event: WindowEvent);

    fn render(&mut self, view: &wgpu::TextureView, device: &wgpu::Device, queue: &wgpu::Queue);

    fn update_texture(&mut self, device: &wgpu::Device, texture: &wgpu::Texture, width: u32, height: u32) {
        // Default implementation does nothing.
    }

    fn update_animation(&mut self, delta: f32) {
        // Default implementation does nothing.
    }

    fn update_skin_buffer(&mut self, device: &wgpu::Device, queue: &wgpu::Queue) {
        // Default implementation does nothing.
    }

    fn update_render_param(&mut self, device: &wgpu::Device, param: RenderParamKind, value: f32) {
        // Default implementation does nothing.
    }

    fn update_light_param(&mut self, param: LightParamKind, value: f32) {
        // Default implementation does nothing.
    }

    fn update_render_params_buffer(&mut self, queue: &wgpu::Queue, size: &PhysicalSize<u32>) {
        // Default implementation does nothing.
    }

    fn load_assets(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, asset: Vec<u8>, instances: Option<Vec<InstanceNode>>, canvas_size: &mut PhysicalSize<u32>) {
        // Default implementation does nothing.
    }

    fn create_hdr_passes(&mut self, device: &wgpu::Device) -> (
        Option<RenderTexturePass>, // sky_hdr_pass
        Option<Pass>, // forward_hdr_pass
        Option<Pass>, // transmission_hdr_pass
        Option<BloomComputePass>, // bloom_pass
        Option<KawaseBlurComputePass>, // kawase_blur_pass
        Option<RenderTexturePass>, // tonemap_pass
    ) {
        (
            None, // sky_hdr_pass
            None, // forward_hdr_pass
            None, // transmission_hdr_pass
            None, // bloom_pass
            None, // kawase_blur_pass
            None, // tonemap_pass
        )
    }
}

#[wasm_bindgen]
pub fn update_framework_animation(delta: f32)
{
    //log::info!("[{}] update_framework_animation invoked", delta);
    EVENT_PROXY.with(|cell| {
        if let Some(proxy) = &*cell.borrow() {
            let _ = proxy.send_event(UserEvent::UpdateAnimation(delta));
        }
    });
}

#[wasm_bindgen]
pub fn set_render_param(kind: RenderParamKind, value: f32)
{
    //log::info!("set_{:?}: {}", kind, value);
    EVENT_PROXY.with(|cell| {
        if let Some(proxy) = &*cell.borrow() {
            let _ = proxy.send_event(UserEvent::SendRenderParam(kind, value));
        }
    });
}

#[wasm_bindgen]
pub fn set_light_param(kind: LightParamKind, value: f32)
{
    //log::info!("set_{:?}: {}", kind, value);
    EVENT_PROXY.with(|cell| {
        if let Some(proxy) = &*cell.borrow() {
            let _ = proxy.send_event(UserEvent::SendLightParam(kind, value));
        }
    });
}

#[wasm_bindgen]
pub fn send_image_bitmap_to_wgpu(rgba_bytes: &[u8], width: u32, height: u32)
{
    let data = rgba_bytes.to_vec();
    EVENT_PROXY.with(|cell| {
        if let Some(proxy) = &*cell.borrow() {
            let _ = proxy.send_event(UserEvent::SendTexture(data, width, height));
        }
    });
}

#[derive(Deserialize)]
struct VisitorPemResponse {
    encryptedPemB64: String,
    ivB64: String,
}

/*
async fn fetch_visitor_private_pem() -> String {
    // 1. X25519 キーペア生成
    let mut rng = OsRng;
    let client_secret = EphemeralSecret::random_from_rng(&mut rng);
    let client_pub = PublicKey::from(&client_secret);
    let client_pub_b64 = general_purpose::STANDARD.encode(client_pub.as_bytes());

    // 2. サーバーに mode=request で送信
    let res = Request::get(&format!(
        "/api/auth/get_asset_key?mode=request&clientPubKeyB64={}&delay=5",
        client_pub_b64
    ))
    .send()
    .await
    .unwrap();

    let json: serde_json::Value = res.json().await.unwrap();
    let request_id = json["requestId"].as_str().unwrap();

    // 3. mode=fetch で取得
    loop {
        let res = Request::get(&format!(
            "/api/auth/get_asset_key?mode=fetch&requestId={}",
            request_id
        ))
        .send()
        .await
        .unwrap();

        if res.status() == 202 {
            let json: serde_json::Value = res.json().await.unwrap();
            let available_at = json["availableAt"].as_u64().unwrap();
            let delay_ms = (available_at as i64 - js_sys::Date::now() as i64).max(0);
            //wasm_timer::Delay::new(std::time::Duration::from_millis(delay_ms as u64)).await.unwrap();
            sleep(Duration::from_millis(delay_ms as u64)).await;
            continue;
        }

        let json: VisitorPemResponse = res.json().await.unwrap();
        let encrypted = general_purpose::STANDARD.decode(&json.encryptedPemB64).unwrap();
        let iv = general_purpose::STANDARD.decode(&json.ivB64).unwrap();

        // 4. サーバーの X25519 公開鍵から共有鍵を生成 (ここはサーバー側と同じ計算式)
        let server_pub_bytes = ...; // サーバーから渡す
        let server_pub = PublicKey::from(server_pub_bytes);
        let shared_secret = client_secret.diffie_hellman(&server_pub);

        // 5. AES-GCM で復号
        let cipher = Aes256Gcm::new_from_slice(shared_secret.as_bytes()).unwrap();
        let nonce = Nonce::from_slice(&iv);
        let plaintext = cipher.decrypt(nonce, encrypted.as_ref()).unwrap();
        return String::from_utf8(plaintext).unwrap();
    }
}
 */

#[derive(Deserialize)]
struct BlobUrl {
    url: String,
}

#[derive(Deserialize)]
struct AssetKey {
    s: String,
}

fn create_dummy_sky(device: &wgpu::Device, queue: &wgpu::Queue) -> wgpu::Texture {
    let size = wgpu::Extent3d {
        width: 1,
        height: 1,
        depth_or_array_layers: 6,
    };

    let white: [u8; 24] = [
        255, 255, 255, 0,
        255, 255, 255, 0,
        255, 255, 255, 0,
        255, 255, 255, 0,
        255, 255, 255, 0,
        255, 255, 255, 0,
    ];

    let texture = device.create_texture_with_data(
        queue,
        &wgpu::TextureDescriptor {
            size,
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: None,
            view_formats: &[],
        },
        // KTX2 stores mip levels in mip major order.
        wgpu::util::TextureDataOrder::MipMajor,
        &white,
    );

    texture
}

pub async fn download_sky(device: &wgpu::Device, queue: &wgpu::Queue, sky_asset_dir: String) -> Result<wgpu::Texture, UnpadError> {
    let device_features = device.features();
    let skybox_format = if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_ASTC) {
        log::info!("Using astc");
        wgpu::TextureFormat::Astc {
            block: AstcBlock::B4x4,
            channel: AstcChannel::UnormSrgb,
        }
    } else if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_ETC2) {
        log::info!("Using etc2");
        wgpu::TextureFormat::Etc2Rgb8A1UnormSrgb
    } else if device_features.contains(wgpu::Features::TEXTURE_COMPRESSION_BC) {
        log::info!("Using bc7");
        wgpu::TextureFormat::Bc7RgbaUnormSrgb
    } else {
        log::info!("Using rgba8");
        wgpu::TextureFormat::Rgba8UnormSrgb
    };

    let IMAGE_SIZE = 256;

    let size = wgpu::Extent3d {
        width: IMAGE_SIZE,
        height: IMAGE_SIZE,
        depth_or_array_layers: 6,
    };

    let layer_size = wgpu::Extent3d {
        depth_or_array_layers: 1,
        ..size
    };
    let max_mips = layer_size.max_mips(wgpu::TextureDimension::D2);

    log::info!("Copying {skybox_format:?} skybox images of size {IMAGE_SIZE}, {IMAGE_SIZE}, 6 with {max_mips} mips to gpu");

    let ktx2file_name = match skybox_format {
        wgpu::TextureFormat::Astc {
            block: AstcBlock::B4x4,
            channel: AstcChannel::UnormSrgb,
        } => "astc.ktx2",
        wgpu::TextureFormat::Etc2Rgb8A1UnormSrgb => "etc2.ktx2",
        wgpu::TextureFormat::Bc7RgbaUnormSrgb => "bc7.ktx2",
        wgpu::TextureFormat::Rgba8UnormSrgb => "rgba8.ktx2",
        _ => unreachable!(),
    };

    // sas url を取得
    let mut res = Request::get(format!("/api/auth/get_blob_url?name={}/{}", sky_asset_dir, ktx2file_name).as_str())
        .send()
        .await
        .unwrap();
    let blob_url: BlobUrl = res.json().await.unwrap();
    let url = blob_url.url;
    //log::info!("SAS url: {:?}", &url);

    // 実際のファイルをダウンロード
    res = Request::get(&url)
        .send()
        .await
        .unwrap();

    let bytes = res.binary().await.unwrap();

    // Readerを作成
    let reader = ktx2::Reader::new(bytes.as_slice()).unwrap();

    // ヘッダー情報を取得
    let header = reader.header();

    /*
    log::info!("KTX2 header:");
    log::info!("  format: {:?}", header.format);
    log::info!("  pixel_width: {}", header.pixel_width);
    log::info!("  pixel_height: {}", header.pixel_height);
    log::info!("  pixel_depth: {}", header.pixel_depth);
    log::info!("  layer_count: {}", header.layer_count);
    log::info!("  face_count: {}", header.face_count);
    log::info!("  level_count: {}", header.level_count);
    log::info!("  supercompression_scheme: {:?}", header.supercompression_scheme);
     */

    let mut image = Vec::with_capacity(reader.data().len());
    for level in reader.levels() {
        image.extend_from_slice(level.data);
    }

    let texture = device.create_texture_with_data(
        queue,
        &wgpu::TextureDescriptor {
            size,
            mip_level_count: header.level_count,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: skybox_format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            label: None,
            view_formats: &[],
        },
        // KTX2 stores mip levels in mip major order.
        wgpu::util::TextureDataOrder::MipMajor,
        &image,
    );

    Ok(texture)
}

#[derive(Deserialize)]
pub struct InstanceTransform {
    pub translation: [f32; 3],
    pub rotation: [f32; 4],
    pub scale: [f32; 3],
}

#[derive(Deserialize)]
pub struct InstanceNode {
    pub parent: String,
    pub source: String,
    pub transforms: Vec<InstanceTransform>,
}

pub async fn download_instances_json(main_asset: Option<String>) -> Result<Vec<InstanceNode>, UnpadError> {
    let filename = match main_asset {
        Some(name) => name.replace(".enc", ""),
        None => {
            log::error!("No main asset specified");
            return Err(UnpadError);
        }
    };
    // sas url を取得
    let res = Request::get(format!("/api/auth/get_blob_url?name={}_instances.json.gz", filename).as_str())
        .send()
        .await
        .unwrap();

    let blob_url: BlobUrl = res.json().await.unwrap();
    let url = blob_url.url;
    log::info!("instances_json SAS url: {:?}", &url);

    // 実際のファイルをダウンロード
    let res = Request::get(&url)
        .send()
        .await
        .map_err(|e| {
            log::error!("Failed to get blob for instances json: {:?}", e);
            UnpadError
        })?;

    if res.status() != 200 {
        log::error!("Failed to get blob for instances json: HTTP {}", res.status());
        return Err(UnpadError);
    }

    let instance_nodes: Vec<InstanceNode> = res.json::<Vec<InstanceNode>>().await.unwrap();
    Ok(instance_nodes)
}

async fn download_and_decrypt(main_asset: Option<String>) -> Result<Vec<u8>, UnpadError> {
    // sas url を取得
    let filename = match main_asset {
        Some(name) => name,
        None => {
            log::error!("No main asset specified");
            return Err(UnpadError);
        }
    };
    if let Ok(mut res) = Request::get(format!("/api/auth/get_blob_url?name={}", filename).as_str())
        .send()
        .await
    {
        let blob_url: BlobUrl = res.json().await.unwrap();
        let url = blob_url.url;
        log::info!("SAS url: {:?}", &url);

        // 実際のファイルをダウンロード
        res = Request::get(&url)
            .send()
            .await
            .unwrap();
        let enc_data = res.binary().await.unwrap();

        //log::info!("Downloaded glTF content: {:?}", &enc_data);

        decrypt_data(enc_data).await
    } else {
        Err(UnpadError)
    }
}

async fn decrypt_data(data: Vec<u8>) -> Result<Vec<u8>, UnpadError> {
    // 先頭16バイトをIVとして取り出す
    let (iv, ciphertext) = data.split_at(16);

    // asset key を取得
    let res = Request::get(format!("/api/auth/get_asset_key").as_str())
        .send()
        .await
        .unwrap();
    let asset_key: AssetKey = res.json().await.unwrap();

    let key_hex = asset_key.s;
    let key_vec = hex::decode(key_hex).map_err(|_| UnpadError)?;

    // 型を [u8; 32] に変換
    let key: [u8; 32] = key_vec.try_into().map_err(|_| UnpadError)?;
    let iv: [u8; 16] = iv.try_into().expect("IV must be 16 bytes");

    let mut buf = ciphertext.to_vec();

    let plaintext = Aes256CbcDec::new(&key.into(), &iv.into())
        .decrypt_padded_mut::<Pkcs7>(&mut buf)?;

    Ok(plaintext.to_vec())
}

pub fn convert_rgba_bytes_to_texture(rgba_bytes: &[u8], texture_format: wgpu::TextureFormat, width: u32, height: u32) -> Option<wgpu::Texture> {
    //log::info!("set_rgba_bytes invoked width:{}, height:{}", width, height);
    let (device, queue) = CONTEXT.with(|cell| {
        let context = cell.borrow();
        let device = context.as_ref().unwrap().device.clone();
        let queue = context.as_ref().unwrap().queue.clone();
        (device, queue)
    });

    let texture_extent = wgpu::Extent3d {
        width: width,
        height: height,
        depth_or_array_layers: 1,
    };

    let texture = Some(device.create_texture(&wgpu::TextureDescriptor {
        label: None,
        size: texture_extent,
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: texture_format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    }));

    queue.write_texture(
        texture
        .as_ref()
        .unwrap()
        .as_image_copy(),
        &rgba_bytes, // imageData.data をそのまま &[u8] にしたもの
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(4 * width), // RGBAなので4倍
            rows_per_image: Some(height),
        },
        wgpu::Extent3d {
            width,
            height,
            depth_or_array_layers: 1,
        },
    );

    texture
}

#[derive(Debug, Clone)]
pub enum UserEvent {
    SendTexture(Vec<u8>, u32, u32),
    UpdateAnimation(f32),
    SendRenderParam(RenderParamKind,f32),
    SendLightParam(LightParamKind,f32),
}

thread_local! {
    static EVENT_PROXY: RefCell<Option<EventLoopProxy<UserEvent>>> = RefCell::new(None);
}

struct EventLoopWrapper {
    event_loop: EventLoop<UserEvent>,
    window: Arc<Window>,
}

impl EventLoopWrapper {
    pub fn new(title: &str) -> Self {
        //let event_loop = EventLoop::new().unwrap();
        let event_loop = EventLoopBuilder::<UserEvent>::with_user_event().build().unwrap();

        EVENT_PROXY.with(|cell| {
            *cell.borrow_mut() = Some(event_loop.create_proxy());
        });

        let mut builder = winit::window::WindowBuilder::new();
        #[cfg(target_arch = "wasm32")]
        {
            use wasm_bindgen::JsCast;
            use winit::platform::web::WindowBuilderExtWebSys;
            let canvas = web_sys::window()
                .unwrap()
                .document()
                .unwrap()
                .get_element_by_id("wgpu-canvas")
                .unwrap()
                .dyn_into::<web_sys::HtmlCanvasElement>()
                .unwrap();
            builder = builder.with_canvas(Some(canvas));
        }
        builder = builder.with_title(title);
        let window = Arc::new(builder.build(&event_loop).unwrap());

        Self { event_loop, window }
    }

}

/// Wrapper type which manages the surface and surface configuration.
///
/// As surface usage varies per platform, wrapping this up cleans up the event loop code.
struct SurfaceWrapper {
    surface: Option<wgpu::Surface<'static>>,
    config: Option<wgpu::SurfaceConfiguration>,
}

impl SurfaceWrapper {
    /// Create a new surface wrapper with no surface or configuration.
    fn new() -> Self {
        Self {
            surface: None,
            config: None,
        }
    }

    /// Called after the instance is created, but before we request an adapter.
    ///
    /// On wasm, we need to create the surface here, as the WebGL backend needs
    /// a surface (and hence a canvas) to be present to create the adapter.
    ///
    /// We cannot unconditionally create a surface here, as Android requires
    /// us to wait until we receive the `Resumed` event to do so.
    fn pre_adapter(&mut self, instance: &Instance, window: Arc<Window>) {
        if cfg!(target_arch = "wasm32") {
            self.surface = Some(instance.create_surface(window).unwrap());
        }
    }

    /// Check if the event is the start condition for the surface.
    fn start_condition(e: &Event<UserEvent>) -> bool {
        match e {
            // On all other platforms, we can create the surface immediately.
            Event::NewEvents(StartCause::Init) => !cfg!(target_os = "android"),
            // On android we need to wait for a resumed event to create the surface.
            Event::Resumed => cfg!(target_os = "android"),
            _ => false,
        }
    }

    /// Called when an event which matches [`Self::start_condition`] is received.
    ///
    /// On all native platforms, this is where we create the surface.
    ///
    /// Additionally, we configure the surface based on the (now valid) window size.
    fn resume(&mut self, context: &ExampleContext, window: Arc<Window>, canvas_size: PhysicalSize<u32>, srgb: bool) {
        /*
        let canvas = web_sys::window()
            .unwrap()
            .document()
            .unwrap()
            .get_element_by_id("wgpu-canvas")
            .unwrap()
            .dyn_into::<web_sys::HtmlCanvasElement>()
            .unwrap();

        let width = canvas.client_width() as u32;
        let height = canvas.client_height() as u32;
         */
        let width = canvas_size.width;
        let height = canvas_size.height;

        // We didn't create the surface in pre_adapter, so we need to do so now.
        if !cfg!(target_arch = "wasm32") {
            self.surface = Some(context.instance.create_surface(window).unwrap());
        }

        // From here on, self.surface should be Some.

        let surface = self.surface.as_ref().unwrap();

        // Get the default configuration,
        let mut config = surface
            .get_default_config(&context.adapter, width, height)
            .expect("Surface isn't supported by the adapter.");
        if srgb {
            // Not all platforms (WebGPU) support sRGB swapchains, so we need to use view formats
            let view_format = config.format.add_srgb_suffix();
            config.view_formats.push(view_format);
        } else {
            // All platforms support non-sRGB swapchains, so we can just use the format directly.
            let format = config.format.remove_srgb_suffix();
            config.format = format;
            config.view_formats.push(format);
        };

        surface.configure(&context.device, &config);
        self.config = Some(config);
    }

    /// Resize the surface, making sure to not resize to zero.
    fn resize(&mut self, context: &ExampleContext, size: PhysicalSize<u32>) {
        //log::info!("Surface resize {size:?}");
        let config = self.config.as_mut().unwrap();
        config.width = size.width.max(1);
        config.height = size.height.max(1);
        let surface = self.surface.as_ref().unwrap();
        surface.configure(&context.device, config);
    }

    /// Acquire the next surface texture.
    fn acquire(&mut self, context: &ExampleContext) -> wgpu::SurfaceTexture {
        let surface = self.surface.as_ref().unwrap();

        match surface.get_current_texture() {
            Ok(frame) => frame,
            // If we timed out, just try again
            Err(wgpu::SurfaceError::Timeout) => surface
                .get_current_texture()
                .expect("Failed to acquire next surface texture!"),
            Err(
                // If the surface is outdated, or was lost, reconfigure it.
                wgpu::SurfaceError::Outdated
                | wgpu::SurfaceError::Lost
                | wgpu::SurfaceError::Other
                // If OutOfMemory happens, reconfiguring may not help, but we might as well try
                | wgpu::SurfaceError::OutOfMemory,
            ) => {
                surface.configure(&context.device, self.config());
                surface
                    .get_current_texture()
                    .expect("Failed to acquire next surface texture!")
            }
        }
    }

    /// On suspend on android, we drop the surface, as it's no longer valid.
    ///
    /// A suspend event is always followed by at least one resume event.
    fn suspend(&mut self) {
        if cfg!(target_os = "android") {
            self.surface = None;
        }
    }

    fn get(&self) -> Option<&'_ Surface<'static>> {
        self.surface.as_ref()
    }

    fn config(&self) -> &wgpu::SurfaceConfiguration {
        self.config.as_ref().unwrap()
    }
}

thread_local! {
    pub static CONTEXT: RefCell<Option<Arc<ExampleContext>>> = RefCell::new(None);
}

#[wasm_bindgen]
pub fn clear_context() {
    CONTEXT.with(|cell| {
        cell.borrow_mut().take();
    });
}

use js_sys::Function;

#[wasm_bindgen]
pub fn resize_by_aspect_ratio_js(aspect: f32) {
    //log::info!("==============> resize_by_aspect_ratio_js invoked aspect {:?}", aspect);
    // JSコードを文字列として Rust に埋め込む（関数本体のみ）
    let js_code = r#"
        const container = document.getElementById("wgpu-container");
        if (!container) return;

        const width = container.clientWidth;           // 横幅は現状のまま
        const height = width / aspect;           // 縦を計算
        container.style.height = `${height}px`;       // 高さだけ変更
    "#;

    // "aspect" 引数で関数を作る
    let func = Function::new_with_args("aspect", js_code);

    // 呼び出す
    func.call1(&JsValue::NULL, &JsValue::from(aspect)).unwrap();
}

macro_rules! define_render_params {
    (
        $( $lower:ident => $upper:ident = $default:expr ),* $(,)?
    ) => {
        #[wasm_bindgen]
        #[derive(Debug, Clone, Copy)]
        pub enum RenderParamKind {
            $( $upper ),*
        }

        #[derive(Debug, Clone)]
        pub struct RenderParams {
            $( pub $lower: f32 ),*
        }

        impl Default for RenderParams {
            fn default() -> Self {
                Self {
                    $( $lower: $default ),*
                }
            }
        }

        impl RenderParamKind {
            pub fn set(self, params: &mut RenderParams, value: f32) {
                match self {
                    $( RenderParamKind::$upper => params.$lower = value ),*
                }
            }

            pub fn get(self, params: &RenderParams) -> f32 {
                match self {
                    $( RenderParamKind::$upper => params.$lower ),*
                }
            }
        }
    };
}

macro_rules! define_light_params {
    (
        $( $lower:ident => $upper:ident = $default:expr ),* $(,)?
    ) => {
        #[wasm_bindgen]
        #[derive(Debug, Clone, Copy)]
        pub enum LightParamKind {
            $( $upper ),*
        }

        #[derive(Debug, Clone)]
        pub struct LightParams {
            $( pub $lower: f32 ),*
        }

        impl Default for LightParams {
            fn default() -> Self {
                Self {
                    $( $lower: $default ),*
                }
            }
        }

        impl LightParamKind {
            pub fn set(self, params: &mut LightParams, value: f32) {
                match self {
                    $( LightParamKind::$upper => params.$lower = value ),*
                }
            }

            pub fn get(self, params: &LightParams) -> f32 {
                match self {
                    $( LightParamKind::$upper => params.$lower ),*
                }
            }
        }
    };
}

define_render_params!(
    use_hdr => UseHdr = 0.0,
    bloom_threshold => BloomThreshold = 1.0,
    bloom_intensity => BloomIntensity = 1.0,
    bloom_env_intensity => BloomEnvIntensity = 2.0,
    sky_brightness => SkyBrightness = 1.0,
    bloom_radius => BloomRadius = 0.5,
    bloom_aspect => BloomAspect = 0.5,
    bloom_weight => BloomWeight = 1.0,
    exposure => Exposure = 1.0,
    saturation => Saturation = 1.0,
    shadow_min_radius => ShadowMinRadius = 0.001,
    shadow_max_radius => ShadowMaxRadius = 0.02,
    clearcoat_weight => ClearcoatWeight = 5.0,
);

define_light_params!(
    shadow_bias_constant => ShadowBiasConstant = 0.0,
    shadow_bias_slope => ShadowBiasSlope = 0.0002,
    shadow_bias_clamp => ShadowBiasClamp = 0.0,
);

/// Context containing global wgpu resources.
struct ExampleContext {
    instance: wgpu::Instance,
    adapter: wgpu::Adapter,
    device: wgpu::Device,
    queue: wgpu::Queue,
}

impl ExampleContext {
    /// Initializes the example context.
    async fn init_async<E: AppMod>(surface: &mut SurfaceWrapper, window: Arc<Window>) -> Self {
        log::info!("Initializing wgpu...");

        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::from_env_or_default());
        surface.pre_adapter(&instance, window);

        let adapter = get_adapter_with_capabilities_or_from_env(
            &instance,
            &E::required_features(),
            &E::required_downlevel_capabilities(),
            &surface.get(),
        )
        .await;
        // Make sure we use the texture resolution limits from the adapter, so we can support images the size of the surface.
        let needed_limits = E::required_limits().using_resolution(adapter.limits());

        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: None,
                required_features: (E::optional_features() & adapter.features())
                    | E::required_features(),
                required_limits: needed_limits,
                memory_hints: wgpu::MemoryHints::MemoryUsage,
                trace: wgpu::Trace::Off,
                /*
                trace: match std::env::var_os("WGPU_TRACE") {
                    Some(path) => wgpu::Trace::Directory(path.into()),
                    None => wgpu::Trace::Off,
                },
                */
                experimental_features: wgpu::ExperimentalFeatures::disabled(),
            })
            .await
            .expect("Unable to find a suitable GPU adapter!");

        Self {
            instance,
            adapter,
            device,
            queue,
        }
    }
}

struct FrameCounter {
    // Instant of the last time we printed the frame time.
    last_printed_instant: web_time::Instant,
    // Number of frames since the last time we printed the frame time.
    frame_count: u32,
}

impl FrameCounter {
    fn new() -> Self {
        Self {
            last_printed_instant: web_time::Instant::now(),
            frame_count: 0,
        }
    }

    fn update(&mut self) {
        self.frame_count += 1;
        let new_instant = web_time::Instant::now();
        let elapsed_secs = (new_instant - self.last_printed_instant).as_secs_f32();
        if elapsed_secs > 1.0 {
            let elapsed_ms = elapsed_secs * 1000.0;
            let frame_time = elapsed_ms / self.frame_count as f32;
            let fps = self.frame_count as f32 / elapsed_secs;
            log::info!("Frame time {frame_time:.2}ms ({fps:.1} FPS)");

            self.last_printed_instant = new_instant;
            self.frame_count = 0;
        }
    }
}

async fn start<E: AppMod>(title: &str, main_asset: Option<String>, sky_asset_dir: Option<String>) {
    init_logger();

    log::debug!(
        "Enabled backends: {:?}",
        wgpu::Instance::enabled_backend_features()
    );

    let window_loop = EventLoopWrapper::new(title);
    let mut surface = SurfaceWrapper::new();
    let context = Arc::new(ExampleContext::init_async::<E>(&mut surface, window_loop.window.clone()).await);

    log::info!("Starting event loop...main_asset ={}", main_asset.as_ref().unwrap_or(&"None".to_string()));

    let asset_download_done: Arc<Mutable<Option<Vec<u8>>>> = Arc::new(Mutable::new(None));
    let sky_download_done: Arc<Mutable<Option<wgpu::Texture>>> = Arc::new(Mutable::new(None));
    let instances_download_done: Arc<Mutable<Option<Vec<InstanceNode>>>> = Arc::new(Mutable::new(None));

    let dec_data = match download_and_decrypt(main_asset.clone()).await {
        Ok(data) => {
            Some(data)
        },
        Err(_) => {
            log::error!("Decryption failed");
            None
        }
    };

    // Decrypted data is available here.
    if let Some(dec_data) = dec_data {
        // send dec_data to load_assets() by asset_download_done
        asset_download_done.lock_mut().replace(dec_data);
    }

    // download skybox texture
    let sky_texture = if let Some(dir) = sky_asset_dir {
            match download_sky(&context.device, &context.queue, dir).await {
                Ok(data) => {
                    Some(data)
                },
                Err(_) => {
                    log::error!("Skybox download failed");
                    None
                }
            }
        } else {
            Some(create_dummy_sky(&context.device, &context.queue))
        };

    // sky texture data is available here.
    if let Some(sky_texture) = sky_texture {
        sky_download_done.lock_mut().replace(sky_texture);
    }

    // download instances json
    let instances_data = match download_instances_json(main_asset).await {
        Ok(data) => {
            data.iter().for_each(|nodes| {
                log::info!("Downloaded {} instance nodes", nodes.transforms.len());
            });
            Some(data)
        },
        Err(_) => {
            log::error!("Instance Data failed");
            None
        }
    };

    // instances data is available here.
    if let Some(data) = instances_data {
        instances_download_done.lock_mut().replace(data);
    }

    CONTEXT.with(|cell| {
        *cell.borrow_mut() = Some(context.clone()); // 新規 CONTEXT
    });

    let mut frame_counter = FrameCounter::new();

    // We wait to create the example until we have a valid surface.
    let mut example = None;

    cfg_if::cfg_if! {
        if #[cfg(target_arch = "wasm32")] {
            use winit::platform::web::EventLoopExtWebSys;
            let event_loop_function = EventLoop::spawn;
        } else {
            let event_loop_function = EventLoop::run;
        }
    }

    //log::info!("Entering event loop...");

    #[cfg_attr(target_arch = "wasm32", expect(clippy::let_unit_value))]
    let _ = (event_loop_function)(
        window_loop.event_loop,
        move |event: Event<UserEvent>, target: &EventLoopWindowTarget<UserEvent>| {
            match event {
                ref e if SurfaceWrapper::start_condition(e) => {
                    //surface.resume(&context, window_loop.window.clone(), E::SRGB);
                    let canvas = web_sys::window()
                        .unwrap()
                        .document()
                        .unwrap()
                        .get_element_by_id("wgpu-canvas")
                        .unwrap()
                        .dyn_into::<web_sys::HtmlCanvasElement>()
                        .unwrap();

                    let canvas_size: PhysicalSize<u32> = PhysicalSize {
                        width: canvas.client_width() as u32,
                        height: canvas.client_height() as u32,
                    };

                    surface.resume(&context, window_loop.window.clone(), canvas_size, true);

                    // If we haven't created the example yet, do so now.
                    if example.is_none() {
                        if let Some(sky_texture) = sky_download_done.lock_mut().take() {
                            //log::info!("ASSET DOWNLOAD DONE. CREATING AppMod...");
                            example = Some(E::init(
                                surface.config(),
                                &context.adapter,
                                &context.device,
                                &context.queue,
                                sky_texture,
                            ));
                        }

                        if let Some(dec_data) = asset_download_done.lock_mut().take() {
                            let mut canvas_size: PhysicalSize<u32> = {
                                let canvas = web_sys::window()
                                    .unwrap()
                                    .document()
                                    .unwrap()
                                    .get_element_by_id("wgpu-canvas")
                                    .unwrap()
                                    .dyn_into::<web_sys::HtmlCanvasElement>()
                                    .unwrap();
                                PhysicalSize {
                                    width: canvas.client_width() as u32,
                                    height: canvas.client_height() as u32,
                                }
                            };
                            // load assets
                            example.as_mut().unwrap().load_assets(
                                &context.device,
                                &context.queue,
                                dec_data,
                                instances_download_done.lock_mut().take(),
                                &mut canvas_size);

                            surface.resume(&context, window_loop.window.clone(), canvas_size, true);
                            surface.resize(&context, canvas_size);
                            
                            example.as_mut().unwrap().resize(
                                &context.device,
                                &context.queue,
                                canvas_size,
                            );
                        }
                    }
                }
                Event::Suspended => {
                    surface.suspend();
                }
                Event::WindowEvent { event, .. } => match event {
                    WindowEvent::Resized(mut size) => {
                        let canvas = web_sys::window()
                            .unwrap()
                            .document()
                            .unwrap()
                            .get_element_by_id("wgpu-canvas")
                            .unwrap()
                            .dyn_into::<web_sys::HtmlCanvasElement>()
                            .unwrap();

                        size.width = canvas.client_width() as u32;
                        size.height = canvas.client_height() as u32;
                        //log::info!("Resized event => Canvas size: {} x {}", size.width, size.height);

                        surface.resize(&context, size);
                        example.as_mut().unwrap().resize(
                            &context.device,
                            &context.queue,
                            size,
                        );

                        window_loop.window.request_redraw();
                    }
                    WindowEvent::KeyboardInput {
                        event:
                            KeyEvent {
                                logical_key: Key::Named(NamedKey::Escape),
                                ..
                            },
                        ..
                    }
                    | WindowEvent::CloseRequested => {
                        target.exit();
                    }
                    #[cfg(not(target_arch = "wasm32"))]
                    WindowEvent::KeyboardInput {
                        event:
                            KeyEvent {
                                logical_key: Key::Character(s),
                                ..
                            },
                        ..
                    } if s == "r" => {
                        println!("{:#?}", context.instance.generate_report());
                    }
                    WindowEvent::RedrawRequested => {
                        // On MacOS, currently redraw requested comes in _before_ Init does.
                        // If this happens, just drop the requested redraw on the floor.
                        //
                        // See https://github.com/rust-windowing/winit/issues/3235 for some discussion
                        if example.is_none() {
                            return;
                        }

                        //frame_counter.update();

                        let frame = surface.acquire(&context);
                        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor {
                            format: Some(surface.config().view_formats[0]),
                            ..wgpu::TextureViewDescriptor::default()
                        });

                        example
                            .as_mut()
                            .unwrap()
                            .render(&view, &context.device, &context.queue);

                        window_loop.window.pre_present_notify();
                        frame.present();

                        window_loop.window.request_redraw();
                    }
                    _ => example.as_mut().unwrap().update(event),
                },
                Event::AboutToWait => {
                    // On some platforms, we need to wait for the next event to be ready.
                    // This is especially important on Android, where we need to wait for the
                    // next event to be able to render.
                    /*
                    let now = web_sys::window().unwrap().performance().unwrap().now();
                    web_sys::console::log_1(&format!("AboutToWait at {:.2} ms", now).into());
                    */

                    //window_loop.window.request_redraw();
                    //example.as_mut().unwrap().update(WindowEvent::RedrawRequested);
                },
                Event::UserEvent(UserEvent::SendTexture(data, w, h)) => {
                    //log::info!("Event::UserEvent {:?} {} {}", data, w, h);
                    if let Some(texture) = convert_rgba_bytes_to_texture(&data, wgpu::TextureFormat::Rgba8UnormSrgb, w, h) {
                        example.as_mut().unwrap().update_texture(&context.device, &texture, w, h);
                    }
                },
                Event::UserEvent(UserEvent::SendRenderParam(param, value)) => {
                    //log::info!("Event::UserEvent {:?} {}", param, value);
                    example.as_mut().unwrap().update_render_param(&context.device, param, value);
                },
                Event::UserEvent(UserEvent::SendLightParam(param, value)) => {
                    //log::info!("Event::UserEvent {:?} {}", param, value);
                    example.as_mut().unwrap().update_light_param(param, value);
                },
                Event::UserEvent(UserEvent::UpdateAnimation(delta)) => {
                    //log::info!("Event::UserEvent UpdateAnimation {}", delta);
                    example.as_mut().unwrap().update_animation(delta);
                },
                _ => {}
            }
        },
    );
}

pub fn run<E: AppMod>(title: &'static str, main_asset: Option<String>, sky_asset_dir: Option<String>) {
    cfg_if::cfg_if! {
        if #[cfg(target_arch = "wasm32")] {
            // WASMのメインスレッドではなく、ブラウザのスレッドで実行するため spawn_local を使う
            wasm_bindgen_futures::spawn_local(async move { start::<E>(title, main_asset, sky_asset_dir).await })
        } else {
            pollster::block_on(start::<E>(title, main_asset, sky_asset_dir));
        }
    }
}

#[cfg(target_arch = "wasm32")]
/// Parse the query string as returned by `web_sys::window()?.location().search()?` and get a
/// specific key out of it.
pub fn parse_url_query_string<'a>(query: &'a str, search_key: &str) -> Option<&'a str> {
    let query_string = query.strip_prefix('?')?;

    for pair in query_string.split('&') {
        let mut pair = pair.split('=');
        let key = pair.next()?;
        let value = pair.next()?;

        if key == search_key {
            return Some(value);
        }
    }

    None
}


struct MeshInternal {
    positions: Vec<[f32; 3]>,
    normals: Vec<[f32; 3]>,
    indices: Vec<u32>,
}

#[repr(C)]
pub struct MeshData {
    pub positions: *const f32,
    pub normals: *const f32,
    pub indices: *const u32,

    pub vertex_count: u32,
    pub index_count: u32,
}

use thiserror::Error;

#[derive(Debug, Error)]
pub enum LoadError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    #[error("gltf error")]
    Gltf,
}

fn load_gltf_mesh(path: &str) -> Result<MeshInternal, LoadError> {
    let (doc, buffers, _) = gltf::import(path).map_err(|_| LoadError::Gltf)?;

    let mesh = doc.meshes().next().ok_or(LoadError::Gltf)?;
    let primitive = mesh.primitives().next().ok_or(LoadError::Gltf)?;

    let reader = primitive.reader(|b| Some(&buffers[b.index()]));

    let positions: Vec<[f32; 3]> =
        reader.read_positions().ok_or(LoadError::Gltf)?.collect();

    let normals: Vec<[f32; 3]> =
        reader.read_normals().ok_or(LoadError::Gltf)?.collect();

    let indices: Vec<u32> =
        reader.read_indices().ok_or(LoadError::Gltf)?
            .into_u32()
            .collect();

    Ok(MeshInternal {
        positions,
        normals,
        indices,
    })
}

#[no_mangle]
pub extern "C" fn load_mesh(path: *const std::ffi::c_char) -> MeshData {
    let cstr = unsafe { std::ffi::CStr::from_ptr(path) };
    let path = cstr.to_str().unwrap();

    let mesh = load_gltf_mesh(path).unwrap();

    let vertex_count = mesh.positions.len() as u32;
    let index_count = mesh.indices.len() as u32;

    let positions = mesh.positions.as_ptr() as *const f32;
    let normals = mesh.normals.as_ptr() as *const f32;
    let indices = mesh.indices.as_ptr();

    std::mem::forget(mesh); // Rust 側で保持

    MeshData {
        positions,
        normals,
        indices,
        vertex_count,
        index_count,
    }
}

#[no_mangle]
pub extern "C" fn free_mesh(
    positions: *const f32,
    normals: *const f32,
    indices: *const u32,
    vertex_count: u32,
    index_count: u32,
) {
    unsafe {
        drop(Vec::from_raw_parts(
            positions as *mut [f32; 3],
            vertex_count as usize,
            vertex_count as usize,
        ));
        drop(Vec::from_raw_parts(
            normals as *mut [f32; 3],
            vertex_count as usize,
            vertex_count as usize,
        ));
        drop(Vec::from_raw_parts(
            indices as *mut u32,
            index_count as usize,
            index_count as usize,
        ));
    }
}

extern "C" {

struct MeshData
{
    const float* Positions;
    const float* Normals;
    const uint32* Indices;
    uint32 VertexCount;
    uint32 IndexCount;
};

MeshData load_mesh(const char* path);
void free_mesh(
    const float* positions,
    const float* normals,
    const uint32* indices,
    uint32 vertexCount,
    uint32 indexCount
);
}

MeshData Data = load_mesh("D:/test.glb");

TArray<FVector> Vertices;
TArray<FVector> Normals;
TArray<int32> Indices;

Vertices.Reserve(Data.VertexCount);
Normals.Reserve(Data.VertexCount);

for (uint32 i = 0; i < Data.VertexCount; i++)
{
    Vertices.Add(FVector(
        Data.Positions[i * 3 + 0],
        Data.Positions[i * 3 + 1],
        Data.Positions[i * 3 + 2]
    ));
    Normals.Add(FVector(
        Data.Normals[i * 3 + 0],
        Data.Normals[i * 3 + 1],
        Data.Normals[i * 3 + 2]
    ));
}

for (uint32 i = 0; i < Data.IndexCount; i++)
{
    Indices.Add(Data.Indices[i]);
}

// UE メッシュ構築（省略）

// コピー完了後、必ず解放
free_mesh(
    Data.Positions,
    Data.Normals,
    Data.Indices,
    Data.VertexCount,
    Data.IndexCount
);
