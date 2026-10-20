//! Access to real scan test data, which lives outside the repository.

use std::path::PathBuf;

/// Name of the environment variable that points to the test data directory.
pub const ENV_VAR: &str = "DRIFTLESS_TESTDATA";

/// Returns `$DRIFTLESS_TESTDATA/<rel>`, or `None` if the variable is not set.
pub fn path(rel: &str) -> Option<PathBuf> {
    let root = std::env::var_os(ENV_VAR)?;
    Some(PathBuf::from(root).join(rel))
}

#[cfg(test)]
mod tests {
    use super::*;

    // Single test mutating the env var, to avoid races between parallel tests.
    #[test]
    fn path_follows_env_var() {
        std::env::remove_var(ENV_VAR);
        assert_eq!(path("Test/LIDAR_Test.bag"), None);

        std::env::set_var(ENV_VAR, "some-root");
        assert_eq!(
            path("Test/LIDAR_Test.bag"),
            Some(PathBuf::from("some-root").join("Test/LIDAR_Test.bag"))
        );
        std::env::remove_var(ENV_VAR);
    }
}
