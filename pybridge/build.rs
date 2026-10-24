// Links the host's CPython (3.15+: PyUnstable_ThreadState_SetStackProtection).
// PYTHON_LIBDIR: directory with libpython3.15.so (default: the system's).
fn main() {
    println!("cargo:rerun-if-env-changed=PYTHON_LIBDIR");
    if let Ok(dir) = std::env::var("PYTHON_LIBDIR") {
        println!("cargo:rustc-link-search=native={dir}");
        println!("cargo:rustc-link-arg=-Wl,-rpath,{dir}");
    }
    println!("cargo:rustc-link-lib=dylib=python3.15");
}
