import { compareSemver } from './update.service';

describe('compareSemver (SemVer 2.0.0 precedence)', () => {
  it('orders numeric core versions', () => {
    expect(compareSemver('16.4.0', '16.3.0')).toBeGreaterThan(0);
    expect(compareSemver('16.3.0', '16.3.1')).toBeLessThan(0);
    expect(compareSemver('17.0.0', '16.9.9')).toBeGreaterThan(0);
    expect(compareSemver('16.3.0', '16.3.0')).toBe(0);
  });

  it('treats a release as newer than a prerelease of the same core', () => {
    expect(compareSemver('16.3.0', '16.3.0-rc1')).toBeGreaterThan(0);
    expect(compareSemver('16.3.0-rc1', '16.3.0')).toBeLessThan(0);
  });

  it('does NOT consider equal cores equal when one is a prerelease (the old bug)', () => {
    // Old comparator split on /[+-]/ and dropped the suffix, so these were "equal".
    expect(compareSemver('16.3.0', '16.3.0-rc1')).not.toBe(0);
  });

  it('orders two prereleases by dot-separated identifiers', () => {
    expect(compareSemver('16.3.0-rc.2', '16.3.0-rc.1')).toBeGreaterThan(0);
    expect(compareSemver('16.3.0-rc.1', '16.3.0-rc.10')).toBeLessThan(0); // numeric, not lexical
    expect(compareSemver('1.0.0-beta', '1.0.0-alpha')).toBeGreaterThan(0);
  });

  it('ranks alphanumeric identifiers above numeric ones', () => {
    expect(compareSemver('1.0.0-rc', '1.0.0-1')).toBeGreaterThan(0);
  });

  it('a larger set of prerelease identifiers outranks a prefix subset', () => {
    expect(compareSemver('1.0.0-alpha.1', '1.0.0-alpha')).toBeGreaterThan(0);
  });

  it('ignores build metadata and a leading v', () => {
    expect(compareSemver('v16.3.0+build.9', '16.3.0+build.1')).toBe(0);
    expect(compareSemver('v16.4.0', '16.3.0')).toBeGreaterThan(0);
  });

  it('treats a main prerelease build as older than the stable release of the same core', () => {
    // Main builds tag v<ver>-main.<run>; build freshness is handled separately,
    // so a -main prerelease must NOT read as newer than the stable version.
    expect(compareSemver('16.3.0-main.42', '16.3.0')).toBeLessThan(0);
  });

  it('is resilient to malformed input', () => {
    expect(compareSemver('', '')).toBe(0);
    expect(compareSemver('garbage', '0.0.0')).toBe(0);
  });
});
