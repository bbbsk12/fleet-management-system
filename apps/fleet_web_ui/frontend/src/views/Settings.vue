<template>
  <div class="space-y-4 sm:space-y-6">
    <div class="flex items-center justify-between flex-wrap gap-2 sm:gap-4">
      <div>
        <h1 class="text-xl sm:text-2xl font-bold tracking-wide" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('settings.title') }}</h1>
        <p class="text-xs sm:text-sm mt-0.5" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('settings.subtitle') }}</p>
      </div>
      <div class="flex gap-2">
        <button 
          class="px-3 py-2 rounded-lg text-sm transition-colors flex items-center gap-2"
          :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border border-gray-200 text-gray-700 hover:bg-gray-200'"
          @click="resetSettings"
        >
          <RotateCcw class="w-4 h-4" />
          {{ t('settings.resetDefault') }}
        </button>
        <button 
          class="px-3 py-2 rounded-lg text-sm transition-colors flex items-center gap-2 btn-cyber"
          @click="saveSettings"
        >
          <Save class="w-4 h-4" />
          {{ t('settings.saveSettings') }}
        </button>
      </div>
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-2 gap-4 sm:gap-6">
      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <Globe class="w-5 h-5 text-cyber-blue" />
          {{ t('settings.language') }}
        </h3>
        <div class="grid grid-cols-2 gap-3">
          <button
            @click="setLanguage('zh')"
            class="p-4 rounded-lg border transition-all min-h-touch"
            :class="language === 'zh' 
              ? 'bg-cyber-blue/20 border-cyber-blue text-cyber-blue' 
              : (theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:border-gray-500' : 'bg-gray-50 border-gray-200 text-gray-700 hover:border-gray-300')"
          >
            <span class="text-2xl">🇨🇳</span>
            <p class="mt-2 text-sm font-medium">{{ t('settings.chinese') }}</p>
          </button>
          <button
            @click="setLanguage('en')"
            class="p-4 rounded-lg border transition-all min-h-touch"
            :class="language === 'en' 
              ? 'bg-cyber-blue/20 border-cyber-blue text-cyber-blue' 
              : (theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:border-gray-500' : 'bg-gray-50 border-gray-200 text-gray-700 hover:border-gray-300')"
          >
            <span class="text-2xl">🇺🇸</span>
            <p class="mt-2 text-sm font-medium">{{ t('settings.english') }}</p>
          </button>
        </div>
      </div>

      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <Palette class="w-5 h-5 text-cyber-purple" />
          {{ t('settings.theme') }}
        </h3>
        <div class="grid grid-cols-2 gap-3">
          <button
            @click="setTheme('light')"
            class="p-4 rounded-lg border transition-all flex flex-col items-center min-h-touch"
            :class="theme === 'light' 
              ? 'bg-cyber-blue/20 border-cyber-blue text-cyber-blue' 
              : (theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:border-gray-500' : 'bg-gray-50 border-gray-200 text-gray-700 hover:border-gray-300')"
          >
            <Sun class="w-8 h-8 mb-2" />
            <p class="text-sm font-medium">{{ t('settings.lightMode') }}</p>
          </button>
          <button
            @click="setTheme('dark')"
            class="p-4 rounded-lg border transition-all flex flex-col items-center min-h-touch"
            :class="settingsStore.theme === 'dark' 
              ? 'bg-cyber-blue/20 border-cyber-blue text-cyber-blue' 
              : (theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:border-gray-500' : 'bg-gray-50 border-gray-200 text-gray-700 hover:border-gray-300')"
          >
            <Moon class="w-8 h-8 mb-2" />
            <p class="text-sm font-medium">{{ t('settings.darkMode') }}</p>
          </button>
        </div>
      </div>

      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <Network class="w-5 h-5 text-cyber-blue" />
          {{ t('settings.connection') }}
        </h3>
        <div class="space-y-4">
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.rosDomainId') }}</label>
            <input 
              type="number" 
              v-model.number="settings.ros_domain_id" 
              min="0" 
              max="255"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
            <p class="text-xs mt-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('settings.rosDomainIdHint') }}</p>
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.zenohRouter') }}</label>
            <input 
              type="text" 
              v-model="settings.zenoh_router" 
              placeholder="tcp/127.0.0.1:7447"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.wsPort') }}</label>
            <input 
              type="number" 
              v-model.number="settings.ws_port" 
              min="1" 
              max="65535"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.heartbeatInterval') }}</label>
            <input 
              type="number" 
              v-model.number="settings.heartbeat_interval" 
              min="1" 
              max="60"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
        </div>
      </div>
      
      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <ListTodo class="w-5 h-5 text-cyber-purple" />
          {{ t('settings.taskScheduling') }}
        </h3>
        <div class="space-y-4">
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.taskTimeout') }}</label>
            <input 
              type="number" 
              v-model.number="settings.task_timeout" 
              min="30" 
              max="3600"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.maxConcurrentTasks') }}</label>
            <input 
              type="number" 
              v-model.number="settings.max_concurrent_tasks" 
              min="1" 
              max="50"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.taskRetryCount') }}</label>
            <input 
              type="number" 
              v-model.number="settings.task_retry_count" 
              min="0" 
              max="10"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.lowBatteryThreshold') }}</label>
            <input 
              type="number" 
              v-model.number="settings.low_battery_threshold" 
              min="10" 
              max="50"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
            <p class="text-xs mt-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('settings.lowBatteryThresholdHint') }}</p>
          </div>
        </div>
      </div>
      
      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <TrafficCone class="w-5 h-5 text-cyber-orange" />
          {{ t('settings.trafficManagement') }}
        </h3>
        <div class="space-y-4">
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.minSafeDistance') }}</label>
            <input 
              type="number" 
              v-model.number="settings.min_safe_distance" 
              min="0.5" 
              max="5" 
              step="0.1"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.waypointLockRadius') }}</label>
            <input 
              type="number" 
              v-model.number="settings.waypoint_lock_radius" 
              min="0.3" 
              max="2" 
              step="0.1"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.pathPlanningTimeout') }}</label>
            <input 
              type="number" 
              v-model.number="settings.path_planning_timeout" 
              min="5" 
              max="60"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div class="flex items-center justify-between py-2">
            <label class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.conflictDetection') }}</label>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.conflict_detection ? 'bg-cyber-blue' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.conflict_detection = !settings.conflict_detection"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.conflict_detection ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
        </div>
      </div>
      
      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <Monitor class="w-5 h-5 text-cyber-pink" />
          {{ t('settings.interface') }}
        </h3>
        <div class="space-y-4">
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.mapRefreshRate') }}</label>
            <input 
              type="number" 
              v-model.number="settings.map_refresh_rate" 
              min="1" 
              max="30"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-600'">{{ t('settings.logDisplayCount') }}</label>
            <input 
              type="number" 
              v-model.number="settings.log_display_count" 
              min="50" 
              max="1000"
              class="w-full px-4 py-2.5 rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-white' : 'bg-white border border-gray-200 text-gray-900'"
            >
          </div>
          
          <div class="flex items-center justify-between py-2">
            <label class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.enableAnimations') }}</label>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.enable_animations ? 'bg-cyber-blue' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.enable_animations = !settings.enable_animations"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.enable_animations ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
          
          <div class="flex items-center justify-between py-2">
            <label class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.showDebugInfo') }}</label>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.show_debug_info ? 'bg-cyber-blue' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.show_debug_info = !settings.show_debug_info"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.show_debug_info ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
        </div>
      </div>
      
      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <Bell class="w-5 h-5 text-cyber-yellow" />
          {{ t('settings.notifications') }}
        </h3>
        <div class="space-y-2">
          <div class="flex items-center justify-between py-3 px-4 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.taskCompleteNotify') }}</span>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.notify_task_complete ? 'bg-cyber-green' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.notify_task_complete = !settings.notify_task_complete"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.notify_task_complete ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
          
          <div class="flex items-center justify-between py-3 px-4 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.lowBatteryNotify') }}</span>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.notify_low_battery ? 'bg-cyber-green' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.notify_low_battery = !settings.notify_low_battery"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.notify_low_battery ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
          
          <div class="flex items-center justify-between py-3 px-4 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.offlineNotify') }}</span>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.notify_offline ? 'bg-cyber-green' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.notify_offline = !settings.notify_offline"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.notify_offline ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
          
          <div class="flex items-center justify-between py-3 px-4 rounded-lg" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('settings.errorNotify') }}</span>
            <button 
              class="relative w-12 h-6 rounded-full transition-colors"
              :class="settings.notify_errors ? 'bg-cyber-green' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-300')"
              @click="settings.notify_errors = !settings.notify_errors"
            >
              <span 
                class="absolute top-1 w-4 h-4 rounded-full bg-white transition-transform"
                :class="settings.notify_errors ? 'left-7' : 'left-1'"
              ></span>
            </button>
          </div>
        </div>
      </div>
      
      <div class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <h3 class="text-lg font-bold mb-4 flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
          <Info class="w-5 h-5" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'" />
          {{ t('settings.systemInfo') }}
        </h3>
        <div class="space-y-3">
          <div class="flex items-center justify-between py-3 px-4 rounded-lg border" :class="theme === 'dark' ? 'bg-industrial-700/30 border-industrial-600' : 'bg-gray-50 border-gray-200'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('settings.systemVersion') }}</span>
            <span class="text-sm text-cyber-blue font-medium">FleetOS v1.0.0</span>
          </div>
          <div class="flex items-center justify-between py-3 px-4 rounded-lg border" :class="theme === 'dark' ? 'bg-industrial-700/30 border-industrial-600' : 'bg-gray-50 border-gray-200'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('settings.rosVersion') }}</span>
            <span class="text-sm text-cyber-green font-medium">Humble Hawksbill</span>
          </div>
          <div class="flex items-center justify-between py-3 px-4 rounded-lg border" :class="theme === 'dark' ? 'bg-industrial-700/30 border-industrial-600' : 'bg-gray-50 border-gray-200'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('settings.runtime') }}</span>
            <span class="text-sm font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ uptime }}</span>
          </div>
          <div class="flex items-center justify-between py-3 px-4 rounded-lg border" :class="theme === 'dark' ? 'bg-industrial-700/30 border-industrial-600' : 'bg-gray-50 border-gray-200'">
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('settings.lastUpdate') }}</span>
            <span class="text-sm font-medium" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">2024-01-15</span>
          </div>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, reactive, onMounted } from 'vue'
import { storeToRefs } from 'pinia'
import { useFleetStore } from '../stores/fleet'
import { useSettingsStore } from '../stores/settings'
import { 
  RotateCcw, Save, Network, ListTodo, TrafficCone, 
  Palette, Bell, Info, Globe, Monitor, Sun, Moon
} from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const { language, theme } = storeToRefs(settingsStore)
const { setLanguage, setTheme, t } = settingsStore

const settings = reactive({
  ros_domain_id: 0,
  zenoh_router: 'tcp/127.0.0.1:7447',
  ws_port: 8080,
  heartbeat_interval: 5,
  task_timeout: 300,
  max_concurrent_tasks: 10,
  task_retry_count: 3,
  low_battery_threshold: 20,
  min_safe_distance: 1.0,
  waypoint_lock_radius: 0.5,
  path_planning_timeout: 30,
  conflict_detection: true,
  map_refresh_rate: 10,
  log_display_count: 200,
  enable_animations: true,
  show_debug_info: false,
  notify_task_complete: true,
  notify_low_battery: true,
  notify_offline: true,
  notify_errors: true
})

const uptime = ref('')

const defaultSettings = { ...settings }

function saveSettings() {
  localStorage.setItem('fleet_settings', JSON.stringify(settings))
  fleetStore.addLog('success', t('settings.saved'))
  // 同步到后端，供启动脚本/后端在启动时读取（如 ROS_DOMAIN_ID）
  fetch('/api/settings', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      ros_domain_id: settings.ros_domain_id,
      zenoh_router: settings.zenoh_router,
      ws_port: settings.ws_port
    })
  })
    .then(r => r.json().catch(() => ({})))
    .then((data) => {
      const need = data?.restart_required || []
      if (need.length) {
        alert(`${t('settings.saved')}\n需要重启生效: ${need.join(', ')}`)
      } else {
        alert(t('settings.saved'))
      }
    })
    .catch(() => {
      alert(`${t('settings.saved')}\n后端同步失败（仅本地保存）`)
    })
}

function resetSettings() {
  if (confirm(t('settings.confirmReset'))) {
    Object.assign(settings, defaultSettings)
    fleetStore.addLog('info', t('settings.resetSuccess'))
  }
}

onMounted(() => {
  // 先从后端拉取（如果可用），再用本地值兜底
  fetch('/api/settings')
    .then(r => r.ok ? r.json() : Promise.reject())
    .then((data) => {
      if (data?.settings) {
        Object.assign(settings, data.settings)
        localStorage.setItem('fleet_settings', JSON.stringify(settings))
      }
    })
    .catch(() => {
      const saved = localStorage.getItem('fleet_settings')
      if (saved) Object.assign(settings, JSON.parse(saved))
    })
})
</script>
