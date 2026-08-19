pub mod shapes;
pub use shapes::{Shape, Shaper};
pub mod prefab;
pub use prefab::{
    kepler_poinsot,
    platonic_solid,
    polygon,
    prism,
    pyramid,
    wedge,
};
