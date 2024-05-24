use std::fs;

use miette::{IntoDiagnostic, Result};

use crate::{point_cloud::Point, Arguments};

pub fn export_to_ply(arguments: &Arguments, point_cloud: Vec<Point>) -> Result<()> {
    let mut content = format!(
        "ply\nformat ascii 1.0\nelement vertex {}\nproperty float x\nproperty float y\nproperty float z\nproperty uint8 red\nproperty uint8 green\nproperty uint8 blue\nend_header\n",
        point_cloud.len()
    );

    for point in point_cloud.iter() {
        let loc = point.location();
        let col = point.color();

        let r = (col.x * 255.0) as u8;
        let g = (col.y * 255.0) as u8;
        let b = (col.z * 255.0) as u8;

        let point_str = format!("{} {} {} {} {} {}\n", loc.x, loc.y, loc.z, r, g, b);
        content.push_str(&point_str);
    }

    if let Some(out_file) = &arguments.ply_output {
        fs::write(out_file, content).into_diagnostic()?;
    }

    Ok(())
}
