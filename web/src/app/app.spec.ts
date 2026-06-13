import { TestBed } from '@angular/core/testing';
import { App } from './app';
import { TERMS_VERSION } from './services/app-meta';
import { CONSENT_KEY } from './services/rvt-storage-keys';

describe('App', () => {
  beforeEach(async () => {
    localStorage.setItem(CONSENT_KEY, JSON.stringify({
      version: TERMS_VERSION,
      accepted_at: new Date().toISOString()
    }));

    await TestBed.configureTestingModule({
      imports: [App],
    }).compileComponents();
  });

  it('should create the app', () => {
    const fixture = TestBed.createComponent(App);
    const app = fixture.componentInstance;
    expect(app).toBeTruthy();
  });

  it('should host routed content', async () => {
    const fixture = TestBed.createComponent(App);
    await fixture.whenStable();
    const compiled = fixture.nativeElement as HTMLElement;
    expect(compiled.querySelector('router-outlet')).toBeTruthy();
  });
});
