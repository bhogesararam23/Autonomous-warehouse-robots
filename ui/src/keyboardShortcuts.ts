const INTERACTIVE_TARGET_SELECTOR =
  'input, textarea, select, button, a[href], [role], [tabindex], [contenteditable]';

export type PlaybackShortcutAction = 'toggle-play' | 'step-back' | 'step-forward';

type KeyboardShortcutEvent = Pick<
  KeyboardEvent,
  'key' | 'target' | 'altKey' | 'ctrlKey' | 'metaKey' | 'shiftKey' | 'repeat' | 'defaultPrevented'
>;

export function getPlaybackShortcutAction(event: KeyboardShortcutEvent): PlaybackShortcutAction | null {
  if (
    event.defaultPrevented ||
    event.repeat ||
    event.altKey ||
    event.ctrlKey ||
    event.metaKey ||
    event.shiftKey
  ) {
    return null;
  }

  const target = event.target as
    | (EventTarget & {
        closest?: (selectors: string) => Element | null;
        isContentEditable?: boolean;
      })
    | null;

  if (target?.isContentEditable || target?.closest?.(INTERACTIVE_TARGET_SELECTOR)) {
    return null;
  }

  switch (event.key) {
    case ' ':
    case 'Spacebar':
      return 'toggle-play';
    case 'ArrowLeft':
      return 'step-back';
    case 'ArrowRight':
      return 'step-forward';
    default:
      return null;
  }
}
