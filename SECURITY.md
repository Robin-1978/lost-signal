# Security Policy

## Supported versions

Lost Signal is currently an alpha source release. Security fixes are applied to the latest revision on the `main` branch. Older snapshots and local modifications are not maintained as separate supported versions.

## Report a vulnerability privately

Do not open a public issue for a vulnerability that could expose a secret, execute untrusted code, compromise a developer machine, or create a credible hazard when this software is connected to hardware.

Use GitHub's **Report a vulnerability** feature in the Security tab of the repository. Include:

- the affected revision and platform;
- the smallest reproducible example;
- the expected and actual behavior;
- the likely impact;
- any known mitigation.

Please avoid including credentials, personal data, or unrelated system information. Maintainers will acknowledge a valid report, investigate it, and coordinate disclosure and a fix according to its impact.

## Physical-robot boundary

Lost Signal is an educational simulator. It is not a safety controller and has not been validated for unsupervised operation on physical robots. Anyone connecting it to hardware must add independent emergency stopping, command limits, watchdogs, collision protection, workspace isolation, and hardware-specific validation. Never test around people, traffic, animals, or valuable property without an appropriate safety system and responsible supervision.
