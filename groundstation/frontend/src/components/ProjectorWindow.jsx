import { useEffect, useState, useRef } from 'react';
import { createPortal } from 'react-dom';

// A second browser window, opened on a chosen display, holding nothing but the
// C-scan grid — the image that gets projected onto the wall the scan was swept
// over.
//
// There is no browser API that "sends a view to a display" the way screen
// SHARING picks a capture source; sharing is capture, and this is output. The
// closest primitive is the Window Management API (`window.getScreenDetails()`),
// which enumerates the attached displays and lets a window be opened on one of
// them and full-screened there. The caller does the enumeration (it needs a
// user gesture and it drives the panel's picker); this component takes the
// resulting rectangle and owns the window.
//
// Content is a React PORTAL rather than a second app: the projector then reads
// the same props as the panel — same grid, same colour limits, same to-scale
// placement — so the two cannot disagree and there is no message channel to
// keep in sync. Everything the operator changes on the panel appears on the
// wall on the next frame.

const WINDOW_NAME = 'cscan-projector';

// Module scope on purpose: this has to survive the mount → unmount → mount that
// React StrictMode performs on every effect in development, and the module
// reload that HMR performs on top of it. See `reclaim` below.
let liveWindow = null;

function openProjectorWindow(t) {
  // `popup=yes` drops the tab strip and toolbar, so the window is already most
  // of the way to a clean output surface before fullscreen is asked for.
  const feat = [
    'popup=yes',
    t ? `left=${Math.round(t.left)}` : null,
    t ? `top=${Math.round(t.top)}` : null,
    `width=${Math.round(t ? t.width : 1280)}`,
    `height=${Math.round(t ? t.height : 800)}`,
  ].filter(Boolean).join(',');

  const win = window.open('', WINDOW_NAME, feat);
  if (!win) return null;
  const doc = win.document;

  // A window opened by NAME can be one that already exists — after an HMR
  // module reload the record above is gone but the window is not — so the
  // document is only furnished once. Without this it would collect a second
  // root and a second copy of the stylesheet on every hot reload.
  let el = doc.querySelector('[data-projector-root]');
  if (!el) {
    doc.title = 'C-Scan Projection';
    doc.body.style.cssText = 'margin:0;padding:0;background:#000;overflow:hidden;cursor:none;';
    doc.documentElement.style.cssText = 'background:#000;';

    // The portal renders components written against this app's stylesheet, so
    // the stylesheet has to exist in the other document too. Cloned rather than
    // re-linked because in dev Vite injects <style> tags with no URL to point at.
    for (const node of document.querySelectorAll('style, link[rel="stylesheet"]')) {
      doc.head.appendChild(node.cloneNode(true));
    }

    el = doc.createElement('div');
    el.setAttribute('data-projector-root', '');
    el.style.cssText = 'position:fixed;inset:0;background:#000;';
    doc.body.appendChild(el);

    const hint = doc.createElement('div');
    hint.setAttribute('data-projector-hint', '');
    hint.textContent = 'Click anywhere for full screen · Esc to exit · close this window to stop';
    hint.style.cssText = [
      'position:fixed', 'left:0', 'right:0', 'bottom:0', 'padding:10px',
      'font:11px ui-monospace,monospace', 'letter-spacing:.08em', 'text-align:center',
      'color:#666', 'background:#000', 'pointer-events:none', 'z-index:10',
    ].join(';');
    doc.body.appendChild(hint);
  }

  return { win, doc, el, hint: doc.querySelector('[data-projector-hint]'), closeTimer: null };
}

export default function ProjectorWindow({ target, onClose, rootRef, children }) {
  const [container, setContainer] = useState(null);
  // The window is opened from an effect with no dependencies, so the target is
  // read through a ref rather than closed over.
  const targetRef = useRef(target);
  const closeRef = useRef(onClose);
  closeRef.current = onClose;

  useEffect(() => {
    // Reclaim a window whose teardown is still pending, rather than opening a
    // second one.
    //
    // This is what makes the component work in development. React StrictMode
    // mounts, unmounts and remounts every effect, and a popup does not survive
    // that naively: the click's user activation is spent on the FIRST
    // window.open, so after the cleanup closes it the browser REFUSES the
    // second. Measured, before this: open #1 returned a window, open #2
    // returned null, and the panel reported a blocked popup — so the projector
    // never opened at all under `npm run dev`, while the production build (no
    // StrictMode double-invoke) worked fine. That difference is exactly what
    // makes it look like a random browser problem rather than a bug here.
    //
    // The close is therefore deferred by a tick, and a remount inside that tick
    // takes the same window back. A genuine unmount has no remount to cancel
    // it, so the window still closes.
    let rec = liveWindow;
    if (rec && rec.win && !rec.win.closed) {
      clearTimeout(rec.closeTimer);
      rec.closeTimer = null;
    } else {
      rec = openProjectorWindow(targetRef.current);
      if (!rec) { closeRef.current('blocked'); return undefined; }
      liveWindow = rec;
    }

    const { win, doc, el, hint } = rec;
    if (rootRef) rootRef.current = el;

    // Mount the content BEFORE asking for fullscreen. requestFullscreen can
    // throw synchronously (a rejected promise is the documented failure, but a
    // bad `screen` member is a TypeError), and a throw here would abort the
    // effect with the portal never mounted — an empty black window and no way
    // to close it from the panel.
    setContainer(el);

    // Fullscreen needs user activation, and whether the gesture that opened
    // this window still counts by the time the window exists is not something
    // to rely on — so it is attempted immediately AND rearmed on any click in
    // the projector window, with a hint saying so. `screen` is passed when the
    // caller had a real ScreenDetailed, which is what puts fullscreen on the
    // projector rather than on whichever display the window happens to touch;
    // if that is refused, plain fullscreen still lands on the display the
    // window was opened on.
    const goFullscreen = () => {
      const t = targetRef.current;
      const attempt = (opts) => {
        try {
          const p = doc.documentElement.requestFullscreen(opts);
          return p && p.catch ? p : Promise.resolve();
        } catch (e) {
          return Promise.reject(e);
        }
      };
      const base = { navigationUI: 'hide' };
      const first = t && t.screen ? attempt({ ...base, screen: t.screen }) : attempt(base);
      first.catch(() => { attempt(base).catch(() => {}); });
    };
    const onFsChange = () => {
      if (hint) hint.style.display = doc.fullscreenElement ? 'none' : '';
    };

    goFullscreen();
    win.addEventListener('click', goFullscreen);
    doc.addEventListener('fullscreenchange', onFsChange);

    // The operator can close the projector window directly; the panel has to
    // follow, or its button would keep claiming a window that is gone.
    const onGone = () => { liveWindow = null; closeRef.current('closed'); };
    win.addEventListener('pagehide', onGone);
    // A popup outlives a reload of its opener, which would leave an orphaned
    // window nothing can control.
    const closeOnUnload = () => { try { win.close(); } catch { /* already gone */ } };
    window.addEventListener('beforeunload', closeOnUnload);

    return () => {
      window.removeEventListener('beforeunload', closeOnUnload);
      win.removeEventListener('pagehide', onGone);
      win.removeEventListener('click', goFullscreen);
      doc.removeEventListener('fullscreenchange', onFsChange);
      if (rootRef) rootRef.current = null;
      // Deliberately no setContainer(null): under StrictMode this cleanup is
      // followed immediately by another mount, and tearing the portal down and
      // back up would flash the projected image for no reason.
      rec.closeTimer = setTimeout(() => {
        if (liveWindow === rec) liveWindow = null;
        try { win.close(); } catch { /* already gone */ }
      }, 0);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  return container ? createPortal(children, container) : null;
}

// Enumerate the attached displays so the operator can pick one.
//
// `getScreenDetails()` is the Window Management API: it prompts for the
// `window-management` permission and must be called from a user gesture, so it
// belongs on a click handler. Where it is unavailable (Firefox, Safari, and
// Chrome without the permission) there is no way to learn about a second
// display at all -- the fallback is to open a normal popup and have the
// operator drag it across, which is what `null` means here.
export async function listDisplays() {
  if (typeof window === 'undefined' || !window.getScreenDetails) return null;
  try {
    const details = await window.getScreenDetails();
    return details.screens.map((s, i) => ({
      id: i,
      label: s.label || `Display ${i + 1}${s.isPrimary ? ' (primary)' : ''}`,
      left: s.availLeft,
      top: s.availTop,
      width: s.availWidth,
      height: s.availHeight,
      isPrimary: !!s.isPrimary,
      isInternal: !!s.isInternal,
      devicePixelRatio: s.devicePixelRatio,
      screen: s,
    }));
  } catch {
    // Permission denied, or no Window Management support behind the feature
    // check. Either way there is nothing to choose from.
    return null;
  }
}
