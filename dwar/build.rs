//! Compiles the SangamIO and DhruvaSLAM protobufs straight from the daemons'
//! own crates, so dwar can't drift from them.

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let protos = ["../sangamio/proto/sangamio.proto", "../dhruva-slam/proto/dhruva.proto"];
    let mut config = prost_build::Config::new();
    config.type_attribute(".", "#[allow(clippy::enum_variant_names)]");
    config.compile_protos(&protos, &["../sangamio/proto/", "../dhruva-slam/proto/"])?;
    for p in protos {
        println!("cargo:rerun-if-changed={}", p);
    }
    // The web UI is embedded from web/dist (built by `npm run build` in web/).
    println!("cargo:rerun-if-changed=web/dist");
    if !std::path::Path::new("web/dist/index.html").exists() {
        return Err("web/dist is missing: run `npm ci && npm run build` in dwar/web first".into());
    }
    Ok(())
}
