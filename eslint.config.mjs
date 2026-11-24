import js from '@eslint/js';
import { defineConfig } from 'eslint/config';
import globals from 'globals';
import tseslint from 'typescript-eslint';

export default defineConfig([
  {
    ignores: [
      'node_modules/**',
      'web/**',
      'www/**',
      'assets/**',
      'src-tauri/**',
      'packaging/**',
      'android/**',
      'dist/**',
      'playwright-report/**',
      'test-results/**',
      '*.html',
    ],
  },
  {
    files: ['scripts/**/*.mjs', 'tests/**/*.mjs', 'eslint.config.mjs'],
    extends: [js.configs.recommended],
    languageOptions: {
      ecmaVersion: 2022,
      sourceType: 'module',
      globals: {
        ...globals.node,
      },
    },
  },
  {
    files: ['tests/**/*.ts', 'playwright.config.ts', 'capacitor.config.ts'],
    extends: [js.configs.recommended, tseslint.configs.recommended],
    languageOptions: {
      globals: {
        ...globals.node,
        ...globals.browser,
      },
    },
  },
]);
