// ============================================================================
//  国际化入口 —— locales/index.js
//  功能：统一管理多语言翻译资源，提供语言列表获取及消息查询接口
// ============================================================================

import zh from './zh'
import en from './en'

/** 多语言消息字典，key 为语言代码，value 为对应的翻译键值对 */
const messages = { zh, en }

/**
 * 获取系统支持的所有语言代码列表
 * @returns {string[]} 语言代码数组（如 ['zh', 'en']）
 */
export function getSupportedLocales() {
  return Object.keys(messages)
}

/**
 * 获取指定语言的翻译消息字典
 * 若指定语言不存在，则默认返回中文（zh）翻译
 * @param {string} locale - 语言代码（如 'zh', 'en'）
 * @returns {object} 翻译消息键值对字典
 */
export function getMessages(locale) {
  return messages[locale] || messages.zh
}

export default messages
