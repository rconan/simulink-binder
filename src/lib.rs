//! A procedural macro to import a Simulink C model into Rust

use proc_macro::TokenStream;
use proc_macro2::{Ident, Span};
use quote::quote;
use regex::Regex;
use std::io::{BufRead, BufReader, Cursor};
use syn::parse::{Parse, ParseStream};
use syn::{parse_macro_input, Result};

// proc macro inputs argument
struct Args {
    // the name of the Simulink model
    model: syn::Ident,
    // the path to the Simulink C model header file
    header: syn::LitStr,
}
impl Parse for Args {
    // inputs argument parser
    fn parse(input: ParseStream) -> Result<Self> {
        let model = input.parse()?;
        input.parse::<syn::Token![,]>()?;
        let header = input.parse()?;
        Ok(Self { model, header })
    }
}
// Simulink inputs/outputs
#[derive(Debug)]
struct IO {
    // i/o variable name
    pub name: String,
    // i/o variable identifier
    pub ident: Ident,
    // i/o variable size
    pub size: usize,
}
impl IO {
    // Creates a new IO
    fn new(name: &str, size: &str) -> Self {
        Self {
            name: name.to_string(),
            ident: Ident::new(name, Span::call_site()),
            size: size.parse().unwrap(),
        }
    }
    // Rust enum variant
    fn variant(&self) -> Ident {
        Ident::new(&self.name.replace("_", ""), Span::call_site())
    }
    // Rust variable
    fn var(&self) -> Ident {
        Ident::new(&self.name.to_lowercase(), Span::call_site())
    }
}
// Parse the Simulink C header file to extract inputs and outputs variables
fn parse_io(lines: &mut std::io::Lines<BufReader<Cursor<String>>>, io: &str) -> Option<Vec<IO>> {
    let re = Regex::new(r"_T (?P<name>\w+)\[(?P<size>\d+)\]").unwrap();
    match lines.next() {
        Some(Ok(line)) if line.starts_with("typedef struct") => {
            println!("| {}:", io);
            let mut io_data = vec![];
            while let Some(Ok(line)) = lines.next() {
                if line.contains(io) {
                    break;
                } else {
                    let caps = re.captures(&line).unwrap();
                    let rs_type_name = &caps["name"].replace("_", "");
                    let rs_var_name = &caps["name"].to_lowercase();
                    println!(
                        "|  - {:<10}: {:>6} => ({:>10} : {:<8})",
                        &caps["name"], &caps["size"], rs_var_name, rs_type_name
                    );
                    io_data.push(IO::new(&caps["name"], &caps["size"]))
                }
            }
            Some(io_data)
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
    let Args { model, header } = parse_macro_input!(input);
    println!("Parsing Simulink model {}:", model);
    let file = Cursor::new(header.value());
    let reader = BufReader::new(file);
    let mut lines = reader.lines();

    let mut model_inputs = vec![];
    let mut model_outputs = vec![];
    while let Some(Ok(line)) = lines.next() {
        if line.contains("External inputs") {
            if let Some(ref mut io) = parse_io(&mut lines, "ExtU") {
                model_inputs.append(io);
            }
        }
        if line.contains("External outputs") {
            if let Some(ref mut io) = parse_io(&mut lines, "ExtY") {
                model_outputs.append(io);
            }
        }
    }
    let sim_u: Vec<_> = model_inputs.iter().map(|i| i.ident.clone()).collect();
    let size_u: Vec<_> = model_inputs.iter().map(|i| i.size).collect();
    let enum_u: Vec<_> = model_inputs.iter().map(|i| i.variant()).collect();
    let var_u: Vec<_> = model_inputs.iter().map(|i| i.var()).collect();
    let sim_y: Vec<_> = model_outputs.iter().map(|o| o.ident.clone()).collect();
    let size_y: Vec<_> = model_outputs.iter().map(|o| o.size).collect();
    let enum_y: Vec<_> = model_outputs.iter().map(|i| i.variant()).collect();
    let var_y: Vec<_> = model_outputs.iter().map(|i| i.var()).collect();
    let code = quote! {
    pub trait Simulink {
            fn initialize(&mut self);
            fn __step__(&self);
            fn terminate(&self);
    }
        paste::paste!{
            /// Simulink external input (U)
            #[repr(C)]
            #[allow(non_snake_case)]
            #[derive(Debug)]
            struct [<ExtU_ #model _T>] {
            #(#sim_u: [f64;#size_u]),*
        }}
        paste::paste!{
            /// Simulink external output (Y)
            #[repr(C)]
            #[allow(non_snake_case)]
            #[derive(Debug)]
            struct [<ExtY_ #model _T>] {
            #(#sim_y: [f64;#size_y]),*
        }}

        paste::paste!{
        extern "C" {
            fn [<#model _initialize>]();
            fn [<#model _step>]();
            fn [<#model _terminate>]();
            static mut [<#model _U>]: [<ExtU_ #model _T>];
            static mut [<#model _Y>]: [<ExtY_ #model _T>];
        }}
        /// Controller inputs U
        #[derive(Debug)]
        pub enum U<'a> {
            #(#enum_u(&'a mut [f64; #size_u])),*
        }
        impl<'a> std::ops::Index<usize> for U<'a> {
            type Output = f64;
            fn index(&self, index: usize) -> &Self::Output {
                match self {
                    #(U::#enum_u(data) => &data[index]),*
                }
            }
        }
        impl<'a> std::ops::IndexMut<usize> for U<'a> {
            fn index_mut(&mut self, index: usize) -> &mut Self::Output {
                match self {
                    #(U::#enum_u(data) => &mut data[index]),*
                }
            }
        }
        /// Controller outputs Y
        #[derive(Debug)]
        pub enum Y<'a> {
            #(#enum_y(&'a mut [f64; #size_y])),*
        }
        impl<'a> std::ops::Index<usize> for Y<'a> {
            type Output = f64;
            fn index(&self, index: usize) -> &Self::Output {
                match self {
                    #(Y::#enum_y(data) => &data[index]),*
                }
            }
        }
        impl<'a> From<&Y<'a>> for Vec<f64> {
            fn from(y: &Y<'a>) -> Vec<f64> {
                match y {
                    #(Y::#enum_y(data) => data.to_vec()),*
                }
            }
        }
        /// Controller
        pub struct Controller<'a> {
            #(pub #var_u: U<'a>,)*
            #(pub #var_y: Y<'a>,)*
        }
        paste::paste!{
        impl<'a> Controller<'a> {
            /// Creates a new controller
            pub fn new() -> Self {
                let mut this = unsafe {
                    Self {
                        #(#var_u: U::#enum_u(&mut [<#model _U>].#sim_u),)*
                        #(#var_y: Y::#enum_y(&mut [<#model _Y>].#sim_y),)*
                    }
                };
                this.initialize();
                this
            }
        }}
        paste::paste! {
        impl<'a> Simulink for Controller<'a> {
            fn initialize(&mut self) {
                unsafe {
                    [<#model _initialize>]();
                }
            }
            fn __step__(&self) {
                unsafe {
                    [<#model _step>]();
                }
            }
            fn terminate(&self) {
                unsafe {
                    [<#model _terminate>]();
                }
            }
        }
        }
        impl<'a> Drop for Controller<'a> {
            fn drop(&mut self) {
                self.terminate()
            }
        }
        impl<'a> Iterator for &Controller<'a> {
            type Item = ();
            fn next(&mut self) -> Option<Self::Item> {
                self.__step__();
                Some(())
            }
        }
        impl<'a> Iterator for Controller<'a> {
            type Item = ();
            fn next(&mut self) -> Option<Self::Item> {
                self.__step__();
                Some(())
            }
        }
    };
    code.into()
}
