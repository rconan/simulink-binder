pub use simulink_binder::import;
use std::{
    env, fs,
    path::{Path, PathBuf},
};

pub fn build() {
    let lib = env::var("CARGO_PKG_NAME").unwrap();
    let sys = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap()).join("sys");

    let mut cc_builder = cc::Build::new();
    let mut bindings = bindgen::builder();

    if let Ok(entries) = fs::read_dir(sys) {
        for entry in entries {
            if let Ok(entry) = entry {
                let file_name = entry.path();
                if let Some(extension) = file_name.extension() {
                    match extension.to_str() {
                        Some("c") => {
                            cc_builder.file(file_name);
                        }
                        Some("h") => {
                            bindings = bindings.header(
                                file_name
                                    .to_str()
                                    .expect(&format!("{:?} conversion to str failed", file_name)),
                            );
                            println!("cargo:rerun-if-changed={:}", file_name.to_str().unwrap());
                        }
                        _ => (),
                    }
                }
            }
        }
    }

    cc_builder.compile(lib.as_str());

    println!("cargo:rustc-link-search=native=lib{}", lib);
    println!("cargo:rustc-link-lib={}", lib);

    let bindings = bindings
        .parse_callbacks(Box::new(bindgen::CargoCallbacks))
        .generate()
        .expect("Unable to generate bindings");

    let out_path = PathBuf::from(std::env::var("OUT_DIR").unwrap());
    bindings
        .write_to_file(out_path.join("bindings.rs"))
        .expect("Couldn't write bindings!");
}
