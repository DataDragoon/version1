import { useState, useEffect, useRef, useCallback } from 'react';

const RECONNECT_INTERVAL = 500;

export function useWebSocket(url, onMessage) {
  const [status, setStatus] = useState('disconnected');
  const wsRef = useRef(null);
  const reconnectTimer = useRef(null);
  const onMessageRef = useRef(onMessage);
  onMessageRef.current = onMessage;
  const urlRef = useRef(url);
  urlRef.current = url;

  const connect = useCallback(() => {
    const cur = wsRef.current;
    if (cur && (cur.readyState === WebSocket.OPEN || cur.readyState === WebSocket.CONNECTING)) {
      return;
    }

    const target = urlRef.current;
    if (!target) return;

    clearTimeout(reconnectTimer.current);
    reconnectTimer.current = null;

    setStatus('reconnecting');
    const ws = new WebSocket(target);
    wsRef.current = ws;

    ws.onopen = () => {
      if (wsRef.current !== ws) { ws.close(); return; }
      console.log(`[ws] connected to ${target}`);
      setStatus('connected');
    };

    ws.onmessage = (event) => {
      try {
        const msg = JSON.parse(event.data);
        onMessageRef.current?.(msg);
      } catch {}
    };

    ws.onclose = (ev) => {
      if (wsRef.current === ws) {
        wsRef.current = null;
        console.warn(`[ws] disconnected from ${target} (code=${ev.code})`);
        setStatus('disconnected');
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = setTimeout(connect, RECONNECT_INTERVAL);
      }
    };

    ws.onerror = () => ws.close();
  }, []);

  const disconnect = useCallback(() => {
    const ws = wsRef.current;
    if (ws) {
      wsRef.current = null;
      clearTimeout(reconnectTimer.current);
      reconnectTimer.current = null;
      ws.close();
    }
    setStatus('disconnected');
  }, []);

  useEffect(() => {
    return () => {
      const ws = wsRef.current;
      if (ws) {
        wsRef.current = null;
        clearTimeout(reconnectTimer.current);
        reconnectTimer.current = null;
        ws.close();
      }
    };
  }, []);

  const send = useCallback((msg) => {
    const ws = wsRef.current;
    if (ws?.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify(msg));
    }
  }, []);

  return { status, send, connect, disconnect };
}
