import zh from './zh'
import en from './en'

const messages = { zh, en }

/**
 * Get all supported locale codes.
 * @returns {string[]}
 */
export function getSupportedLocales() {
  return Object.keys(messages)
}

/**
 * Get translation messages for a given locale.
 * @param {string} locale - Locale code (e.g. 'zh', 'en')
 * @returns {object}
 */
export function getMessages(locale) {
  return messages[locale] || messages.zh
}

export default messages
