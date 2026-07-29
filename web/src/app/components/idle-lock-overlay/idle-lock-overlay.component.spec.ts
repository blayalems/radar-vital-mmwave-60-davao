import { signal } from '@angular/core';
import { TestBed } from '@angular/core/testing';
import { MatDialog } from '@angular/material/dialog';
import { beforeEach, describe, expect, it, vi } from 'vitest';

import { AuthService } from '../../services/auth.service';
import { FirstRunService } from '../../services/first-run.service';
import { IdleLockService } from '../../services/idle-lock.service';
import { ServerLifecycleService } from '../../services/server-lifecycle.service';
import { StateService } from '../../services/state.service';
import { IdleLockOverlayComponent } from './idle-lock-overlay.component';

describe('IdleLockOverlayComponent first-run display name', () => {
  const createProfile = vi.fn();

  beforeEach(async () => {
    createProfile.mockReset();

    await TestBed.configureTestingModule({
      imports: [IdleLockOverlayComponent],
      providers: [
        {
          provide: IdleLockService,
          useValue: {
            locked: signal(false),
            unlock: vi.fn()
          }
        },
        {
          provide: StateService,
          useValue: {
            triggerHaptic: vi.fn()
          }
        },
        {
          provide: AuthService,
          useValue: {
            bootstrapping: signal(true),
            currentOperator: signal(null),
            isLocked: signal(true),
            loading: signal(false),
            loginError: signal<string | null>(null),
            profiles: signal([]),
            createProfile,
            loadProfiles: vi.fn()
          }
        },
        {
          provide: ServerLifecycleService,
          useValue: {
            platform: signal('web')
          }
        },
        {
          provide: FirstRunService,
          useValue: {
            consentRequired: signal(false)
          }
        },
        {
          provide: MatDialog,
          useValue: {
            open: vi.fn()
          }
        }
      ]
    }).compileComponents();
  });

  it('exposes the 64-character limit and associated guidance', async () => {
    const fixture = TestBed.createComponent(IdleLockOverlayComponent);
    await fixture.whenStable();

    const input = fixture.nativeElement.querySelector(
      'input[placeholder="e.g. Dr. Sarah Connor"]'
    ) as HTMLInputElement;
    const requirements = fixture.nativeElement.querySelector('#displayNameRequirements');
    const counter = fixture.nativeElement.querySelector('#displayNameCounter');

    expect(input.maxLength).toBe(64);
    expect(input.getAttribute('aria-describedby')).toBe(
      'displayNameRequirements displayNameCounter'
    );
    expect(requirements.textContent).toContain('3–64 characters');
    expect(counter.textContent).toContain('64 remaining');
  });

  it('caps an oversized paste and updates the remaining-character counter', async () => {
    const fixture = TestBed.createComponent(IdleLockOverlayComponent);
    await fixture.whenStable();

    const input = fixture.nativeElement.querySelector(
      'input[placeholder="e.g. Dr. Sarah Connor"]'
    ) as HTMLInputElement;
    input.value = 'A'.repeat(80);
    input.dispatchEvent(new Event('input', { bubbles: true }));
    await fixture.whenStable();

    expect(fixture.componentInstance.displayName).toHaveLength(64);
    expect(input.value).toHaveLength(64);
    expect(fixture.nativeElement.querySelector('#displayNameCounter').textContent).toContain(
      '0 remaining'
    );
  });

  it('rejects an oversized value at submit even if it bypasses the input element', async () => {
    const fixture = TestBed.createComponent(IdleLockOverlayComponent);
    const component = fixture.componentInstance;
    component.displayName = 'A'.repeat(65);
    component.initials = 'AB';
    component.onboardingPin = '123456';

    expect(component.isOnboardingValid()).toBe(false);
    await component.submitOnboarding();
    expect(createProfile).not.toHaveBeenCalled();
  });
});
