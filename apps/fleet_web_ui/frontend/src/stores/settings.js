// ============================================================================
//  系统设置状态管理 —— stores/settings.js
//  功能：Pinia 状态仓库，管理语言、主题等全局配置，
//        并自动持久化到 localStorage，提供国际化翻译函数
// ============================================================================

import { defineStore } from 'pinia'
import { ref, watch } from 'vue'
import { getMessages, getSupportedLocales } from '../locales'

export const useSettingsStore = defineStore('settings', () => {

  // ========================================================================
  //  状态定义（含持久化初始化）
  // ========================================================================

  /** 当前语言，默认中文（zh），从 localStorage 恢复 */
  const language = ref(localStorage.getItem('language') || 'zh')
  /** 当前主题，默认浅色（light），从 localStorage 恢复 */
  const theme = ref(localStorage.getItem('theme') || 'light')

  // ========================================================================
  //  响应式侦听器（自动持久化）
  // ========================================================================

  /** 语言变更时自动保存到 localStorage 并更新 html 标签 lang 属性 */
  watch(language, (val) => {
    localStorage.setItem('language', val)
    document.documentElement.setAttribute('lang', val)
  })

  /** 主题变更时自动保存到 localStorage、更新 data-theme 属性及 CSS class */
  watch(theme, (val) => {
    localStorage.setItem('theme', val)
    document.documentElement.setAttribute('data-theme', val)
    updateThemeClass(val)
  })

  // ========================================================================
  //  内部方法
  // ========================================================================

  /**
   * 更新页面根元素的 dark CSS class
   * 用于切换 Tailwind / 自定义 CSS 的暗色模式样式
   * @param {string} theme - 主题名称（'dark' 或 'light'）
   */
  function updateThemeClass(theme) {
    if (theme === 'dark') {
      document.documentElement.classList.add('dark')
    } else {
      document.documentElement.classList.remove('dark')
    }
  }

  // ========================================================================
  //  公共方法
  // ========================================================================

  /**
   * 设置语言
   * @param {string} lang - 语言代码（如 'zh', 'en'）
   */
  function setLanguage(lang) {
    language.value = lang
  }

  /**
   * 设置主题
   * @param {string} newTheme - 主题名称（'dark' 或 'light'）
   */
  function setTheme(newTheme) {
    theme.value = newTheme
  }

  /**
   * 翻译：根据当前语言获取指定 key 对应文本
   * 若未找到对应翻译，则回退返回 key 本身
   * @param {string} key - 翻译键（如 'common.save'）
   * @returns {string} 翻译后的文本字符串
   */
  function t(key) {
    const messages = getMessages(language.value)
    return messages[key] || key
  }

  // ========================================================================
  //  初始化执行
  // ========================================================================

  // 应用启动时执行一次，确保 HTML 根元素属性与设置同步
  updateThemeClass(theme.value)
  document.documentElement.setAttribute('lang', language.value)
  document.documentElement.setAttribute('data-theme', theme.value)

  // ========================================================================
  //  导出接口
  // ========================================================================

  return {
    language,
    theme,
    setLanguage,
    setTheme,
    t,
    getSupportedLocales
  }
})
