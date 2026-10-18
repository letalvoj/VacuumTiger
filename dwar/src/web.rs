//! The web UI, embedded from `web/dist` (built by `npm run build` in `web/`).
//! Unknown paths serve `index.html`, so client-side routes work on reload.

use axum::http::{HeaderMap, HeaderValue, StatusCode, Uri, header};
use axum::response::{IntoResponse, Response};
use rust_embed::Embed;

#[derive(Embed)]
#[folder = "web/dist/"]
struct Assets;

pub async fn asset(uri: Uri, req_headers: HeaderMap) -> Response {
    let path = uri.path().trim_start_matches('/');
    let path = if path.is_empty() { "index.html" } else { path };
    // Client-side routes (no extension) get the app shell; a missing asset is a 404.
    let is_route = !path.rsplit('/').next().unwrap_or("").contains('.');
    let Some((file, name)) = Assets::get(path)
        .map(|f| (f, path))
        .or_else(|| is_route.then(|| Assets::get("index.html").map(|f| (f, "index.html"))).flatten())
    else {
        return (StatusCode::NOT_FOUND, "not found").into_response();
    };
    let etag = format!("\"{}\"", hex(&file.metadata.sha256_hash()[..8]));
    // Vite fingerprints everything under assets/; the shell revalidates.
    let cache = if name.starts_with("assets/") {
        "public, max-age=31536000, immutable"
    } else {
        "no-cache"
    };
    let mut headers = HeaderMap::new();
    headers.insert(header::CACHE_CONTROL, HeaderValue::from_static(cache));
    if let Ok(v) = HeaderValue::from_str(&etag) {
        headers.insert(header::ETAG, v);
    }
    if req_headers.get(header::IF_NONE_MATCH).and_then(|v| v.to_str().ok()) == Some(etag.as_str()) {
        return (StatusCode::NOT_MODIFIED, headers).into_response();
    }
    if let Ok(v) = HeaderValue::from_str(file.metadata.mimetype()) {
        headers.insert(header::CONTENT_TYPE, v);
    }
    (headers, file.data).into_response()
}

fn hex(bytes: &[u8]) -> String {
    bytes.iter().map(|b| format!("{:02x}", b)).collect()
}
