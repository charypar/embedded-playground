use std::env;
use std::fs::File;
use std::io::Write;
use std::path::PathBuf;

fn main() {
    if let Some(arch) = env::var_os("CARGO_CFG_TARGET_ARCH") {
        if arch != "arm" {
            return;
        }

        let out = &PathBuf::from(env::var_os("OUT_DIR").unwrap());

        println!("Cargo build: {:?}", out);

        File::create(out.join("memory.x"))
            .unwrap()
            .write_all(include_bytes!("memory.x"))
            .unwrap();

        println!("cargo:rustc-link-search={}", out.display());
        println!("cargo:rerun-if-changed=memory.x");
    }

    println!("cargo:rerun-if-changed=build.rs");
}
