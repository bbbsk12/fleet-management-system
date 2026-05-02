import { defineStore } from 'pinia'
import { ref, watch } from 'vue'
import { getMessages, getSupportedLocales } from '../locales'

export const useSettingsStore = defineStore('settings', () => {
  const language = ref(localStorage.getItem('language') || 'zh')
  const theme = ref(localStorage.getItem('theme') || 'light')

  watch(language, (val) => {
    localStorage.setItem('language', val)
    document.documentElement.setAttribute('lang', val)
  })

  watch(theme, (val) => {
    localStorage.setItem('theme', val)
    document.documentElement.setAttribute('data-theme', val)
    updateThemeClass(val)
  })

  function updateThemeClass(theme) {
    if (theme === 'dark') {
      document.documentElement.classList.add('dark')
    } else {
      document.documentElement.classList.remove('dark')
    }
  }

  function setLanguage(lang) {
    language.value = lang
  }

  function setTheme(newTheme) {
    theme.value = newTheme
  }

  /**
   * Translate a key using the current locale.
   * Falls back to the key itself if no translation found.
   * @param {string} key - Translation key (e.g. 'common.save')
   * @returns {string}
   */
  function t(key) {
    const messages = getMessages(language.value)
    return messages[key] || key
  }

  updateThemeClass(theme.value)
  document.documentElement.setAttribute('lang', language.value)
  document.documentElement.setAttribute('data-theme', theme.value)

  return {
    language,
    theme,
    setLanguage,
    setTheme,
    t,
    getSupportedLocales
  }
})
