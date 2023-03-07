//! A procedural macro to import a Simulink C model into Rust

use proc_macro::TokenStream;
use proc_macro2::{Ident, Span};
use quote::quote;
use regex::Regex;
use std::env;
use std::fs::File;
use std::io::{BufRead, BufReader};
use std::path::Path;
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Result};

// proc macro inputs argument
struct Args {
    // the name of the Simulink model
    model: syn::Ident,
    // the path to the Simulink C model header file
    // header: syn::LitStr,
}
impl Parse for Args {
    // inputs argument parser
    fn parse(input: ParseStream) -> Result<Self> {
        let model = input.parse()?;
        // input.parse::<syn::Token![,]>()?;
        // let header = input.parse()?;
        Ok(Self { model })
    }
}
// Simulink inputs/outputs
#[derive(Debug, Default)]
struct IO {
    // i/o variable name
    pub name: String,
    // i/o variable size
    pub size: Option<usize>,
}
impl IO {
    // Creates a new IO
    fn new(name: &str, size: Option<&str>) -> Self {
        Self {
            name: name.to_string(),
            size: size.and_then(|s| s.parse().ok()),
        }
    }
    // Rust variable
    fn var(&self) -> Ident {
        Ident::new(&self.name, Span::call_site())
    }
}
#[derive(Debug, Default)]
struct List(Vec<IO>);
impl List {
    fn quote(&self) -> proc_macro2::TokenStream {
        self.0
            .iter()
            .fold(proc_macro2::TokenStream::default(), |t, io| {
                let var = io.var();
                if let Some(size) = io.size {
                    quote! {
                        #t
                        #var: [0f64;#size],
                    }
                } else {
                    quote! {
                        #t
                        #var: 0f64,
                    }
                }
            })
    }
}

// Parse the Simulink C header file to extract inputs and outputs variables
fn parse_io(lines: &mut std::io::Lines<BufReader<File>>, io: &str) -> Option<List> {
    let re = Regex::new(r"_T (?P<name>\w+)(?:\[(?P<size>\d+)\])?").unwrap();
    match lines.next() {
        Some(Ok(line)) if line.starts_with("typedef struct") => {
            println!("| {}:", io);
            let mut io_data = vec![];
            while let Some(Ok(line)) = lines.next() {
                if line.contains(io) {
                    break;
                } else {
                    if let Some(caps) = re.captures(&line) {
                        let size = caps.name("size").map(|m| m.as_str());
                        println!("|  - {:<22}: {:>5}", &caps["name"], size.unwrap_or("1"),);
                        io_data.push(IO::new(&caps["name"], size))
                    }
                }
            }
            Some(List(io_data))
        }
        _ => None,
    }
}

/// Writes the Rust wrapper for a Simulink C model
///
/// # Examples
///
///```
/// import!(M1HPloadcells)
///```
#[proc_macro]
pub fn import(input: TokenStream) -> TokenStream {
    let Args { model } = parse_macro_input!(input);
    println!("Parsing Simulink model {}:", model);
    let path = Path::new(&env::var("CARGO_MANIFEST_DIR").unwrap())
        .join("sys")
        .join(model.to_string())
        .with_extension("h");
    let file = File::open(&path).expect(&format!("file {:?} not found", path));
    let reader = BufReader::new(file);
    let mut lines = reader.lines();

    let mut model_inputs = List::default();
    let mut model_outputs = List::default();
    let mut model_states = List::default();
    while let Some(Ok(line)) = lines.next() {
        if line.contains("External inputs") {
            if let Some(io) = parse_io(&mut lines, "ExtU") {
                model_inputs = io;
            }
        }
        if line.contains("External outputs") {
            if let Some(io) = parse_io(&mut lines, "ExtY") {
                model_outputs = io;
            }
        }
        if line.contains("Block states") {
            if let Some(io) = parse_io(&mut lines, "DW") {
                model_states = io;
            }
        }
    }

    let var_u = model_inputs.quote();
    let var_y = model_outputs.quote();
    let var_s = model_states.quote();

    let code = quote! {
        include!(concat!(env!("OUT_DIR"), "/bindings.rs"));

        paste::paste!{
        /// Simulink controller wrapper
        #[derive(Debug, Clone, Copy, Default)]
        pub struct #model {
            // Inputs Simulink structure
            pub inputs: [<ExtU_ #model _T>],
            // Outputs Simulink structure
            pub outputs: [<ExtY_ #model _T>],
            states: [<DW_ #model _T>],
        }
        impl Default for [<ExtU_ #model _T>] {
            fn default() -> Self {
                Self { #var_u }
            }
        }
        impl Default for [<ExtY_ #model _T>] {
            fn default() -> Self {
                Self { #var_y }
            }
        }
        impl Default for [<DW_ #model _T>] {
            fn default() -> Self {
                Self { #var_s }
            }
        }
        impl #model {
            /// Creates a new controller
            pub fn new() -> Self {
                let mut this: Self = Default::default();
                let mut data: [<RT_MODEL_ #model _T>] = [<tag_RTM_ #model _T>] {
                    dwork: &mut this.states as *mut _,
                };
                unsafe {
                    [< #model _initialize>](
                        &mut data as *mut _,
                        &mut this.inputs as *mut _,
                        &mut this.outputs as *mut _,
                    )
                }
                this
            }
            /// Steps the controller
            pub fn step(&mut self) {
                let mut data: [<RT_MODEL_ #model _T>] = [<tag_RTM_ #model _T>] {
                    dwork: &mut self.states as *mut _,
                };
                unsafe {
                    [<#model _step>](
                        &mut data as *mut _,
                        &mut self.inputs as *mut _,
                        &mut self.outputs as *mut _,
                    )
                }
            }
        }        }
    };
    code.into()
}
