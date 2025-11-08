import React, { useEffect, useMemo, useRef, useState } from 'react';

export type SetupLogLevel = 'info' | 'warn' | 'error' | 'success' | 'debug';

export interface SetupLogEntry {
  t: number; // epoch ms
  level: SetupLogLevel;
  msg: string;
  data?: unknown;
}

interface SetupConsoleProps {
  entries: SetupLogEntry[];
  onClear?: () => void;
  className?: string;
  maxRows?: number;
}

const levelColor: Record<SetupLogLevel, string> = {
  info: 'text-sky-300',
  warn: 'text-yellow-300',
  error: 'text-red-300',
  success: 'text-emerald-300',
  debug: 'text-purple-300',
};

const levelBadgeBg: Record<SetupLogLevel, string> = {
  info: 'bg-sky-900/50 border-sky-700/60',
  warn: 'bg-yellow-900/40 border-yellow-700/60',
  error: 'bg-red-900/40 border-red-700/60',
  success: 'bg-emerald-900/40 border-emerald-700/60',
  debug: 'bg-purple-900/40 border-purple-700/60',
};

function formatTime(ms: number) {
  const d = new Date(ms);
  return d.toLocaleTimeString(undefined, { hour12: false }) + '.' + String(d.getMilliseconds()).padStart(3, '0');
}

const SetupConsole: React.FC<SetupConsoleProps> = ({ entries, onClear, className = '', maxRows = 2000 }) => {
  const [autoScroll, setAutoScroll] = useState(true);
  const scrollRef = useRef<HTMLDivElement>(null);

  const trimmed = useMemo(() => (entries.length > maxRows ? entries.slice(-maxRows) : entries), [entries, maxRows]);

  useEffect(() => {
    if (!autoScroll || !scrollRef.current) return;
    const el = scrollRef.current;
    el.scrollTop = el.scrollHeight;
  }, [trimmed, autoScroll]);

  const handleCopy = async () => {
    const text = trimmed
      .map((e) => `[${formatTime(e.t)}] ${e.level.toUpperCase()}  ${e.msg}${e.data ? ' ' + JSON.stringify(e.data) : ''}`)
      .join('\n');
    try {
      await navigator.clipboard.writeText(text);
    } catch {
      // ignore
    }
  };

  return (
    <div className={`bg-slate-900 border border-slate-700 rounded-lg overflow-hidden ${className}`}>
      <div className="flex items-center justify-between px-3 py-2 bg-slate-800 border-b border-slate-700">
        <div className="text-sm text-slate-300 font-medium">Setup Console</div>
        <div className="flex items-center gap-2">
          <label className="flex items-center gap-1 text-xs text-slate-300 select-none">
            <input type="checkbox" className="accent-indigo-500" checked={autoScroll} onChange={(e) => setAutoScroll(e.target.checked)} />
            Auto-scroll
          </label>
          <button onClick={handleCopy} className="text-xs px-2 py-1 rounded bg-slate-700 hover:bg-slate-600 text-slate-200 border border-slate-600">Copy</button>
          <button onClick={onClear} className="text-xs px-2 py-1 rounded bg-slate-700 hover:bg-slate-600 text-slate-200 border border-slate-600">Clear</button>
        </div>
      </div>

      <div ref={scrollRef} className="h-56 overflow-auto font-mono text-xs p-2 bg-slate-950 text-slate-200">
        {trimmed.length === 0 ? (
          <div className="text-slate-500 px-1 py-2">No logs yet. Actions in RTK setup will appear here.</div>
        ) : (
          <ul className="space-y-1">
            {trimmed.map((e, idx) => (
              <li key={idx} className="flex items-start gap-2">
                <span className="text-slate-500 shrink-0 tabular-nums">[{formatTime(e.t)}]</span>
                <span className={`shrink-0 px-1.5 py-0.5 rounded border text-[10px] mt-0.5 ${levelBadgeBg[e.level]} ${levelColor[e.level]}`}>
                  {e.level.toUpperCase()}
                </span>
                <div className="flex-1 whitespace-pre-wrap break-words">
                  <span className={levelColor[e.level]}>{e.msg}</span>
                  {e.data !== undefined && (
                    <span className="text-slate-400"> {typeof e.data === 'string' ? e.data : JSON.stringify(e.data)}</span>
                  )}
                </div>
              </li>
            ))}
          </ul>
        )}
      </div>
    </div>
  );
};

export default SetupConsole;
