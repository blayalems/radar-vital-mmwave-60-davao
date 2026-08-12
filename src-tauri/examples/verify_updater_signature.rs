//! Fail-closed verifier for the detached signature shipped with the Tauri updater.

use minisign_verify::{PublicKey, Signature};
use std::{env, fs::File, io::Read, path::PathBuf, process};

const PUBLIC_KEY_ENV: &str = "TAURI_UPDATER_PUBLIC_KEY";
const USAGE: &str =
    "usage: verify_updater_signature <staged-installer.exe> <staged-installer.exe.sig>";

fn parse_public_key(raw: &str) -> Result<PublicKey, String> {
    let trimmed = raw.trim();
    if trimmed.is_empty() {
        return Err(format!("{PUBLIC_KEY_ENV} is empty"));
    }

    let result = if trimmed.lines().count() > 1 {
        PublicKey::decode(trimmed)
    } else {
        PublicKey::from_base64(trimmed)
    };
    result.map_err(|error| format!("configured updater public key is invalid: {error}"))
}

fn verify_reader(
    public_key: &PublicKey,
    signature: &Signature,
    mut reader: impl Read,
) -> Result<(), String> {
    let mut verifier = public_key
        .verify_stream(signature)
        .map_err(|error| format!("cannot initialize streaming verification: {error}"))?;
    let mut buffer = [0_u8; 64 * 1024];

    loop {
        let bytes_read = reader
            .read(&mut buffer)
            .map_err(|error| format!("cannot read staged installer: {error}"))?;
        if bytes_read == 0 {
            break;
        }
        verifier.update(&buffer[..bytes_read]);
    }

    verifier
        .finalize()
        .map_err(|error| format!("detached signature does not match staged installer: {error}"))
}

fn run() -> Result<PathBuf, String> {
    let mut args = env::args_os().skip(1);
    let artifact_path = PathBuf::from(args.next().ok_or(USAGE)?);
    let signature_path = PathBuf::from(args.next().ok_or(USAGE)?);
    if args.next().is_some() {
        return Err(USAGE.to_owned());
    }

    let raw_public_key = env::var(PUBLIC_KEY_ENV)
        .map_err(|_| format!("{PUBLIC_KEY_ENV} is not configured"))?;
    let public_key = parse_public_key(&raw_public_key)?;
    let signature = Signature::from_file(&signature_path).map_err(|error| {
        format!(
            "cannot load detached signature {}: {error}",
            signature_path.display()
        )
    })?;
    let artifact = File::open(&artifact_path).map_err(|error| {
        format!(
            "cannot open staged installer {}: {error}",
            artifact_path.display()
        )
    })?;

    verify_reader(&public_key, &signature, artifact)?;
    Ok(artifact_path)
}

fn main() {
    match run() {
        Ok(path) => println!(
            "Verified detached updater signature against the configured public key and {}.",
            path.display()
        ),
        Err(error) => {
            eprintln!("Updater signature verification failed: {error}");
            process::exit(1);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Cursor;

    const TEST_PUBLIC_KEY: &str =
        "RWQf6LRCGA9i53mlYecO4IzT51TGPpvWucNSCh1CBM0QTaLn73Y7GFO3";
    const TEST_SIGNATURE: &str = "untrusted comment: signature from minisign secret key
RUQf6LRCGA9i559r3g7V1qNyJDApGip8MfqcadIgT9CuhV3EMhHoN1mGTkUidF/z7SrlQgXdy8ofjb7bNJJylDOocrCo8KLzZwo=
trusted comment: timestamp:1556193335\tfile:test
y/rUw2y8/hOUYjZU71eHp/Wo1KZ40fGy2VJEDl34XMJM+TX48Ss/17u3IvIfbVR1FkZZSNCisQbuQY+bHwhEBg==";

    fn fixture() -> (PublicKey, Signature) {
        (
            parse_public_key(TEST_PUBLIC_KEY).expect("fixture public key should decode"),
            Signature::decode(TEST_SIGNATURE).expect("fixture signature should decode"),
        )
    }

    #[test]
    fn accepts_signature_for_matching_bytes() {
        let (public_key, signature) = fixture();
        verify_reader(&public_key, &signature, Cursor::new(b"test"))
            .expect("matching signature should verify");
    }

    #[test]
    fn rejects_signature_for_modified_bytes() {
        let (public_key, signature) = fixture();
        let error = verify_reader(&public_key, &signature, Cursor::new(b"tampered"))
            .expect_err("modified bytes must fail verification");
        assert!(error.contains("does not match staged installer"));
    }

    #[test]
    fn rejects_signature_from_different_public_key() {
        let mismatched_public_key = parse_public_key(
            "RWQf6LRCGA9i53mlYecO4IzT51TGPpvWucNSCh1CBM0QTaLn73Y7GFO4",
        )
        .expect("mismatched public key should still decode");
        let signature =
            Signature::decode(TEST_SIGNATURE).expect("fixture signature should decode");
        verify_reader(&mismatched_public_key, &signature, Cursor::new(b"test"))
            .expect_err("a signature from another key must fail verification");
    }

    #[test]
    fn accepts_full_minisign_public_key_format() {
        let formatted =
            format!("untrusted comment: updater public key fixture\n{TEST_PUBLIC_KEY}\n");
        parse_public_key(&formatted).expect("full minisign public key should decode");
    }
}
