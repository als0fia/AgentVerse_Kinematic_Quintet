/**
 * @file src/utils/logger.ts
 * @description Structured Diagnostic & Audit Logger for the Antenna Swarm Control Console.
 *
 * This module implements the application-wide logging policy:
 * 1. Logs every function invocation at the INFO level along with its input parameters.
 * 2. Logs any GenAI API calls with model identifier, prompt, configuration, and output,
 *    while sanitizing and stripping any inline binary/base64 data payloads.
 * 3. Maintains an in-memory bounded ring buffer of structured telemetry log entries so
 *    operators can inspect real-time command and state transitions if needed.
 *
 * Use Cases:
 * - Auditing deterministic robot task dispatches (`TaskIntentV1`), emergency stops, and manual teleop nudges.
 * - Verifying parameter propagation across kinematic calculations and rescue triage evaluations.
 * - Stripping large `inlineData` fields from multimodal payloads prior to console output.
 */

export interface LogRecord {
  id: string;
  timestamp: string;
  category: 'FUNCTION_CALL' | 'GENAI_CALL' | 'SAFETY_GATE';
  name: string;
  details: unknown;
}

const MAX_LOG_HISTORY = 80;
const logBuffer: LogRecord[] = [];

/**
 * Recursively strips any `inlineData` or base64 data blobs from an object before logging.
 * @param payload - Arbitrary parameter or response payload to sanitize.
 * @returns A sanitized copy with inline binary data replaced by a placeholder string.
 */
export function stripInlineData(payload: unknown): unknown {
  if (payload === null || payload === undefined) {
    return payload;
  }
  if (typeof payload !== 'object') {
    return payload;
  }
  if (Array.isArray(payload)) {
    return payload.map((item) => stripInlineData(item));
  }

  const record = payload as Record<string, unknown>;
  const sanitized: Record<string, unknown> = {};
  for (const [key, value] of Object.entries(record)) {
    if (key === 'inlineData' || key === 'inline_data' || key === 'b64_json') {
      sanitized[key] = '[STRIPPED_INLINE_DATA]';
    } else if (typeof value === 'object' && value !== null) {
      sanitized[key] = stripInlineData(value);
    } else {
      sanitized[key] = value;
    }
  }
  return sanitized;
}

/**
 * Logs a function invocation at the INFO level with its function name and parameters.
 * @param functionName - Name of the invoked function.
 * @param params - Dictionary or value of arguments passed to the function.
 */
export function logFunctionCall(functionName: string, params: Record<string, unknown> = {}): void {
  const sanitizedParams = stripInlineData(params);
  const timestamp = new Date().toISOString();
  const entry: LogRecord = {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 7)}`,
    timestamp,
    category: 'FUNCTION_CALL',
    name: functionName,
    details: sanitizedParams,
  };

  logBuffer.unshift(entry);
  if (logBuffer.length > MAX_LOG_HISTORY) {
    logBuffer.pop();
  }

  console.info(`[INFO][FUNCTION_CALL] ${functionName}`, sanitizedParams);
}

/**
 * Logs a GenAI SDK call at the INFO level with model, prompt, config, and output, stripping inlineData.
 * @param model - Model name used for the call.
 * @param prompt - Input prompt or contents passed to the model.
 * @param config - Generation configuration object.
 * @param output - Model response output.
 */
export function logGenAiCall(
  model: string,
  prompt: unknown,
  config: unknown,
  output: unknown
): void {
  const sanitizedRecord = {
    model,
    prompt: stripInlineData(prompt),
    config: stripInlineData(config),
    output: stripInlineData(output),
  };
  const timestamp = new Date().toISOString();
  const entry: LogRecord = {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 7)}`,
    timestamp,
    category: 'GENAI_CALL',
    name: `genai:${model}`,
    details: sanitizedRecord,
  };

  logBuffer.unshift(entry);
  if (logBuffer.length > MAX_LOG_HISTORY) {
    logBuffer.pop();
  }

  console.info(`[INFO][GENAI_CALL] model=${model}`, sanitizedRecord);
}

/**
 * Returns the most recent structured log entries from the in-memory buffer.
 * @param limit - Maximum number of log entries to return.
 * @returns Array of recent LogRecord objects.
 */
export function getRecentLogs(limit = 20): LogRecord[] {
  return logBuffer.slice(0, limit);
}
