//! JSON persistence for the separate organism and motion documents.

use organism_core::{MotionDefinition, Organism};
use std::{
    fs, io,
    path::{Path, PathBuf},
};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ProjectIoError {
    #[error("validation failed: {0:?}")]
    Validation(Vec<String>),
    #[error("organism and motion documents do not refer to the same organism")]
    OrganismIdMismatch,
    #[error("JSON encoding or decoding failed: {0}")]
    Json(#[from] serde_json::Error),
    #[error("filesystem operation failed: {0}")]
    Io(#[from] io::Error),
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ProjectPaths {
    pub organism: PathBuf,
    pub motion: PathBuf,
}

pub fn encode_organism(organism: &Organism) -> Result<String, ProjectIoError> {
    organism.validate().map_err(ProjectIoError::Validation)?;
    Ok(serde_json::to_string_pretty(organism)?)
}

pub fn encode_motion(
    organism: &Organism,
    motion: &MotionDefinition,
) -> Result<String, ProjectIoError> {
    if motion.organism_id != organism.id {
        return Err(ProjectIoError::OrganismIdMismatch);
    }
    motion
        .validate_for(organism)
        .map_err(ProjectIoError::Validation)?;
    Ok(serde_json::to_string_pretty(motion)?)
}

pub fn save_project(
    directory: impl AsRef<Path>,
    organism: &Organism,
    motion: &MotionDefinition,
) -> Result<ProjectPaths, ProjectIoError> {
    // Validate and encode both documents before touching the filesystem.
    let organism_json = encode_organism(organism)?;
    let motion_json = encode_motion(organism, motion)?;

    let directory = directory.as_ref();
    fs::create_dir_all(directory)?;

    let paths = ProjectPaths {
        organism: directory.join("organism.json"),
        motion: directory.join("motion.json"),
    };
    fs::write(&paths.organism, organism_json)?;
    fs::write(&paths.motion, motion_json)?;
    Ok(paths)
}

pub fn load_project(
    directory: impl AsRef<Path>,
) -> Result<(Organism, MotionDefinition), ProjectIoError> {
    let directory = directory.as_ref();
    let organism: Organism =
        serde_json::from_str(&fs::read_to_string(directory.join("organism.json"))?)?;
    let motion: MotionDefinition =
        serde_json::from_str(&fs::read_to_string(directory.join("motion.json"))?)?;

    organism.validate().map_err(ProjectIoError::Validation)?;
    if motion.organism_id != organism.id {
        return Err(ProjectIoError::OrganismIdMismatch);
    }
    motion
        .validate_for(&organism)
        .map_err(ProjectIoError::Validation)?;

    Ok((organism, motion))
}

#[cfg(test)]
mod tests {
    use super::*;
    use organism_core::{MotionDefinition, Organism};

    #[test]
    fn encodes_two_independent_json_documents() {
        let organism = Organism::new("Test organism");
        let motion = MotionDefinition::new(organism.id.clone());

        let organism_json = encode_organism(&organism).unwrap();
        let motion_json = encode_motion(&organism, &motion).unwrap();

        assert!(organism_json.contains("\"format\": \"organism\""));
        assert!(motion_json.contains("\"format\": \"organism_motion\""));
        assert!(!organism_json.contains("\"behaviors\""));
        assert!(!motion_json.contains("\"parts\""));
    }
}
