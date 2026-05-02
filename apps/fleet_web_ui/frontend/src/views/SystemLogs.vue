<template>
  <div class="space-y-4 sm:space-y-6">
    <div class="flex items-center justify-between flex-wrap gap-2 sm:gap-4">
      <div>
        <h1 class="text-xl sm:text-2xl font-bold tracking-wide" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('logs.title') }}</h1>
        <p class="text-xs sm:text-sm mt-0.5" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('logs.subtitle') }}</p>
      </div>
      <div class="flex gap-2">
        <button 
          class="px-3 py-2 rounded-lg text-sm transition-colors flex items-center gap-2"
          :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border border-gray-200 text-gray-600 hover:bg-gray-200'"
          @click="exportLogs"
        >
          <Download class="w-4 h-4" />
          {{ t('logs.exportLogs') }}
        </button>
        <button 
          class="px-3 py-2 rounded-lg bg-cyber-red/20 border border-cyber-red/40 text-cyber-red text-sm hover:bg-cyber-red/30 transition-colors flex items-center gap-2"
          @click="clearLogs"
        >
          <Trash2 class="w-4 h-4" />
          {{ t('logs.clearLogs') }}
        </button>
      </div>
    </div>

    <div class="data-card rounded-xl p-4 neon-border">
      <div class="flex flex-wrap items-center gap-4">
        <div class="flex flex-wrap gap-1 p-1 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-800' : 'bg-gray-100'">
          <button 
            v-for="tab in levelTabs" 
            :key="tab.value"
            class="px-3 py-1.5 rounded-md text-xs font-medium transition-all"
            :class="levelFilter === tab.value 
              ? 'bg-cyber-blue/20 text-cyber-blue border border-cyber-blue/40' 
              : (theme === 'dark' ? 'text-gray-400 hover:text-white' : 'text-gray-500 hover:text-gray-900')"
            @click="levelFilter = tab.value"
          >
            {{ tab.label }}
          </button>
        </div>
        
        <div class="flex-1 min-w-[200px]">
          <div class="relative">
            <Search class="absolute left-3 top-1/2 -translate-y-1/2 w-4 h-4" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'" />
            <input 
              type="text" 
              v-model="searchQuery"
              :placeholder="t('logs.searchPlaceholder')"
              class="w-full pl-10 pr-4 py-2 rounded-lg text-sm focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white placeholder-gray-500' : 'bg-white border border-gray-200 text-gray-900 placeholder-gray-400'"
            >
          </div>
        </div>
      </div>
    </div>
    
    <div class="grid grid-cols-2 lg:grid-cols-4 gap-3 sm:gap-4">
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-700' : 'bg-gray-100'">
            <FileText class="w-5 h-5 sm:w-6 sm:h-6" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('logs.totalLogs') }}</p>
            <p class="text-xl sm:text-2xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ logStats.total }}</p>
          </div>
        </div>
      </div>
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-blue/10">
            <Info class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-blue" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('logs.info') }}</p>
            <p class="text-xl sm:text-2xl font-bold text-cyber-blue">{{ logStats.info }}</p>
          </div>
        </div>
      </div>
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-orange/10">
            <AlertTriangle class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-orange" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('logs.warning') }}</p>
            <p class="text-xl sm:text-2xl font-bold text-cyber-orange">{{ logStats.warning }}</p>
          </div>
        </div>
      </div>
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-red/10">
            <XCircle class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-red" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('logs.error') }}</p>
            <p class="text-xl sm:text-2xl font-bold text-cyber-red">{{ logStats.error }}</p>
          </div>
        </div>
      </div>
    </div>
    
    <div class="data-card rounded-xl p-0 neon-border overflow-hidden">
      <div class="flex items-center justify-between p-4" :class="theme === 'dark' ? 'border-industrial-700' : 'border-gray-200'" style="border-bottom-width: 1px; border-bottom-style: solid;">
        <h3 class="text-lg font-bold flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <FileText class="w-5 h-5 text-cyber-blue" />
          {{ t('logs.logRecords') }}
        </h3>
        <span class="text-sm" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ filteredLogs.length }} {{ t('logs.records') }}</span>
      </div>
      <div class="max-h-[400px] overflow-y-auto">
        <div 
          v-for="(log, index) in filteredLogs" 
          :key="index" 
          class="p-4 cursor-pointer transition-colors"
          :class="theme === 'dark' ? 'border-industrial-700/50 hover:bg-industrial-700/30' : 'border-gray-100 hover:bg-gray-50'"
          style="border-bottom-width: 1px; border-bottom-style: solid;"
          @click="toggleExpand(index)"
        >
          <div class="flex items-start gap-3">
            <span class="text-xs font-mono mt-0.5 shrink-0" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ log.time }}</span>
            <span 
              class="px-2 py-0.5 rounded text-xs font-medium shrink-0"
              :class="logLevelClass(log.level)"
            >
              {{ log.level.toUpperCase() }}
            </span>
            <span v-if="log.source" class="text-xs px-2 py-0.5 rounded shrink-0" :class="theme === 'dark' ? 'bg-industrial-700 text-gray-400' : 'bg-gray-100 text-gray-500'">
              {{ log.source }}
            </span>
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ log.message }}</span>
          </div>
          <div v-if="expandedLog === index && log.details" class="mt-3 p-3 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-800' : 'bg-gray-100'">
            <pre class="text-xs font-mono whitespace-pre-wrap" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ log.details }}</pre>
          </div>
        </div>
        
        <div v-if="filteredLogs.length === 0" class="py-12 text-center">
          <FileText class="w-12 h-12 mx-auto mb-3" :class="theme === 'dark' ? 'text-gray-600' : 'text-gray-300'" />
          <p :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('logs.noLogs') }}</p>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useFleetStore } from '../stores/fleet'
import { useSettingsStore } from '../stores/settings'
import { 
  Download, Trash2, Search, FileText, Info, AlertTriangle, 
  XCircle
} from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const { theme } = storeToRefs(settingsStore)
const { t } = settingsStore

const levelFilter = ref('all')
const searchQuery = ref('')
const expandedLog = ref(null)

const levelTabs = computed(() => [
  { value: 'all', label: t('common.all') },
  { value: 'info', label: t('logs.info') },
  { value: 'warning', label: t('logs.warning') },
  { value: 'error', label: t('logs.error') },
  { value: 'success', label: t('logs.success') }
])

const allLogs = computed(() => {
  return [...fleetStore.logs].sort((a, b) => b.time.localeCompare(a.time))
})

const filteredLogs = computed(() => {
  let logs = allLogs.value
  if (levelFilter.value !== 'all') {
    logs = logs.filter(l => l.level === levelFilter.value)
  }
  if (searchQuery.value) {
    const query = searchQuery.value.toLowerCase()
    logs = logs.filter(l => l.message.toLowerCase().includes(query) || (l.source && l.source.toLowerCase().includes(query)))
  }
  return logs
})

const logStats = computed(() => {
  const logs = allLogs.value
  return {
    total: logs.length,
    info: logs.filter(l => l.level === 'info').length,
    warning: logs.filter(l => l.level === 'warning').length,
    error: logs.filter(l => l.level === 'error').length
  }
})

function logLevelClass(level) {
  const classes = {
    info: 'bg-cyber-blue/20 text-cyber-blue',
    warning: 'bg-cyber-orange/20 text-cyber-orange',
    error: 'bg-cyber-red/20 text-cyber-red',
    success: 'bg-cyber-green/20 text-cyber-green'
  }
  return classes[level] || 'bg-gray-500/20 text-gray-400'
}

function toggleExpand(index) {
  expandedLog.value = expandedLog.value === index ? null : index
}

function exportLogs() {
  const data = JSON.stringify(filteredLogs.value, null, 2)
  const blob = new Blob([data], { type: 'application/json' })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = `fleet_logs_${new Date().toISOString().slice(0, 10)}.json`
  a.click()
  URL.revokeObjectURL(url)
  fleetStore.addLog('success', t('logs.exportSuccess'))
}

function clearLogs() {
  if (confirm(t('logs.confirmClear'))) {
    fleetStore.logs = []
  }
}

</script>
