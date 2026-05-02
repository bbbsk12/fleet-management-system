<template>
  <div class="space-y-4 sm:space-y-6">
    <div class="flex items-center justify-between flex-wrap gap-2 sm:gap-4">
      <div>
        <h1 class="text-xl sm:text-2xl font-bold tracking-wide" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('fleet.title') }}</h1>
        <p class="text-xs sm:text-sm mt-0.5" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('fleet.subtitle') }}</p>
      </div>
      <div class="flex items-center gap-2">
        <button 
          class="px-4 py-2 rounded-lg bg-cyber-red/20 border border-cyber-red/40 text-cyber-red text-sm font-medium hover:bg-cyber-red/30 transition-all flex items-center gap-2"
          @click="emergencyStopAll"
        >
          <StopCircle class="w-4 h-4" />
          {{ t('fleet.emergencyStopAll') }}
        </button>
        <button 
          class="px-4 py-2 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue text-sm font-medium hover:bg-cyber-blue/30 transition-all flex items-center gap-2"
          @click="refreshFleet"
        >
          <RefreshCw class="w-4 h-4" />
          {{ t('fleet.refresh') }}
        </button>
      </div>
    </div>

    <div class="flex flex-wrap items-center gap-4 p-4 rounded-lg border transition-colors" :class="theme === 'dark' ? 'bg-industrial-800/50 border-industrial-600' : 'bg-gray-50 border-gray-200'">
      <div class="flex items-center gap-2 flex-wrap">
        <span class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('common.status') }}:</span>
        <div class="flex gap-1">
          <button 
            v-for="f in filterOptions" 
            :key="f.value"
            class="px-3 py-1.5 rounded-lg text-xs font-medium transition-all"
            :class="filter === f.value 
              ? 'bg-cyber-blue/20 text-cyber-blue border border-cyber-blue/40' 
              : (theme === 'dark' ? 'bg-industrial-700 text-gray-400 hover:text-white' : 'bg-gray-100 text-gray-500 hover:text-gray-900')"
            @click="filter = f.value"
          >
            {{ f.label }}
          </button>
        </div>
      </div>
      <div class="flex-1 min-w-[200px]">
        <input 
          type="text" 
          class="w-full px-4 py-2 border rounded-lg text-sm focus:border-cyber-blue focus:outline-none focus:ring-1 focus:ring-cyber-blue/20"
          :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white placeholder-gray-500' : 'bg-white border-gray-200 text-gray-900 placeholder-gray-400'"
          :placeholder="t('fleet.searchPlaceholder')"
          v-model="searchQuery"
        >
      </div>
    </div>
    
    <div class="grid grid-cols-1 md:grid-cols-2 xl:grid-cols-3 gap-4">
      <div 
        v-for="robot in filteredRobots" 
        :key="robot.id" 
        class="data-card rounded-xl p-4 neon-border"
        :class="{ 'opacity-60 border-cyber-red/30': !robot.online || robot.status === 'offline' }"
      >
        <div class="flex items-start justify-between mb-4">
          <div class="flex items-center gap-3">
            <div class="p-2.5 rounded-lg bg-cyber-blue/10">
              <Bot class="w-6 h-6 text-cyber-blue" />
            </div>
            <div>
              <h3 class="font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ robot.id }}</h3>
              <span 
                class="inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-xs font-medium mt-1"
                :class="statusClass(robot.status)"
              >
                <span class="w-1.5 h-1.5 rounded-full" :class="statusDotClass(robot.status)" />
                {{ statusText(robot.status) }}
              </span>
            </div>
          </div>
          <button 
            class="p-2 rounded-lg transition-colors"
            :class="theme === 'dark' ? 'bg-industrial-700 text-gray-400 hover:text-white hover:bg-industrial-600' : 'bg-gray-100 text-gray-500 hover:text-gray-900 hover:bg-gray-200'"
            @click="showRobotDetail(robot)"
          >
            <MoreVertical class="w-4 h-4" />
          </button>
        </div>
        
        <div class="space-y-3 mb-4">
          <div class="flex items-center justify-between">
            <div class="flex items-center gap-2 text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              <Battery class="w-4 h-4" />
              <span>{{ t('fleet.battery') }}</span>
            </div>
            <div class="flex items-center gap-2">
              <div class="w-20 h-2 rounded-full overflow-hidden" :class="theme === 'dark' ? 'bg-industrial-700' : 'bg-gray-200'">
                <div 
                  class="h-full rounded-full transition-all duration-300"
                  :class="batteryClass(robot.battery)"
                  :style="{ width: robot.battery + '%' }"
                />
              </div>
              <span class="text-sm font-mono" :class="batteryTextClass(robot.battery)">{{ robot.battery }}%</span>
            </div>
          </div>
          
          <div class="flex items-center justify-between">
            <div class="flex items-center gap-2 text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              <MapPin class="w-4 h-4" />
              <span>{{ t('fleet.position') }}</span>
            </div>
            <span class="text-sm font-mono" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
              ({{ robot.position?.world_x?.toFixed(2) ?? '-' }}, {{ robot.position?.world_y?.toFixed(2) ?? '-' }})
            </span>
          </div>
          
          <div class="flex items-center justify-between">
            <div class="flex items-center gap-2 text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              <Navigation class="w-4 h-4" />
              <span>{{ t('fleet.location') }}</span>
            </div>
            <span 
              v-if="robot.location_type === 'waypoint'" 
              class="text-sm px-2 py-0.5 rounded-full bg-cyber-green/20 text-cyber-green"
            >
              @{{ robot.current_waypoint }}
            </span>
            <span 
              v-else-if="robot.location_type === 'segment'" 
              class="text-sm px-2 py-0.5 rounded-full bg-cyber-blue/20 text-cyber-blue"
              :title="segmentMotionHint(robot)"
            >
              → {{ robot.current_segment }}
              <span v-if="robot.nav_status" class="opacity-80"> · {{ segmentMotionHint(robot) }}</span>
            </span>
            <span 
              v-else 
              class="text-sm px-2 py-0.5 rounded-full"
              :class="theme === 'dark' ? 'bg-gray-700 text-gray-400' : 'bg-gray-200 text-gray-500'"
            >
              {{ t('common.unknown') }}
            </span>
          </div>
          
          <div class="flex items-center justify-between">
            <div class="flex items-center gap-2 text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              <ListTodo class="w-4 h-4" />
              <span>{{ t('fleet.currentTask') }}</span>
            </div>
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ robot.current_task || t('common.noTask') }}</span>
          </div>
        </div>
        
        <div class="flex gap-2 pt-3 border-t transition-colors" :class="theme === 'dark' ? 'border-industrial-600' : 'border-gray-200'">
          <button 
            class="flex-1 px-3 py-2 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue text-sm font-medium hover:bg-cyber-blue/30 transition-all disabled:opacity-50 disabled:cursor-not-allowed"
            @click="assignTask(robot.id)"
            :disabled="!robot.online || robot.status === 'working'"
          >
            {{ t('fleet.assignTask') }}
          </button>
          <button 
            class="px-3 py-2 rounded-lg border transition-colors disabled:opacity-50"
            :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border-gray-200 text-gray-600 hover:bg-gray-200'"
            @click="recallRobot(robot.id)"
            :disabled="!robot.online"
          >
            <Undo2 class="w-4 h-4" />
          </button>
          <button 
            class="px-3 py-2 rounded-lg bg-cyber-red/20 border border-cyber-red/40 text-cyber-red text-sm hover:bg-cyber-red/30 transition-colors disabled:opacity-50"
            @click="stopRobot(robot.id)"
            :disabled="!robot.online"
          >
            <StopCircle class="w-4 h-4" />
          </button>
          <button 
            class="px-3 py-2 rounded-lg bg-cyber-orange/20 border border-cyber-orange/40 text-cyber-orange text-sm hover:bg-cyber-orange/30 transition-colors"
            @click="removeRobot(robot.id)"
            :title="t('fleet.removeRobot')"
          >
            <UserMinus class="w-4 h-4" />
          </button>
        </div>
      </div>
      
      <div v-if="filteredRobots.length === 0" class="col-span-full">
        <div class="data-card rounded-xl p-12 neon-border text-center">
          <Bot class="w-16 h-16 mx-auto mb-4" :class="theme === 'dark' ? 'text-gray-600' : 'text-gray-300'" />
          <p :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('fleet.noMatchingRobots') }}</p>
        </div>
      </div>
    </div>
    
    <div v-if="selectedRobot" class="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center p-4" @click.self="selectedRobot = null">
      <div class="data-card rounded-xl p-6 neon-border w-full max-w-lg animate-scale-in">
        <div class="flex items-center justify-between mb-6">
          <h2 class="text-xl font-bold flex items-center gap-3" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
            <Bot class="w-6 h-6 text-cyber-blue" />
            {{ selectedRobot.id }} {{ t('fleet.robotDetail') }}
          </h2>
          <button 
            class="p-2 rounded-lg transition-colors"
            :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'"
            @click="selectedRobot = null"
          >
            <X class="w-5 h-5" />
          </button>
        </div>
        
        <div class="grid grid-cols-2 gap-4">
          <div class="p-3 rounded-lg transition-colors" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-xs block mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('common.status') }}</span>
            <span class="text-sm font-medium" :class="statusTextClass(selectedRobot.status)">
              {{ statusText(selectedRobot.status) }}
            </span>
          </div>
          <div class="p-3 rounded-lg transition-colors" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-xs block mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('fleet.battery') }}</span>
            <span class="text-sm font-medium" :class="batteryTextClass(selectedRobot.battery)">
              {{ selectedRobot.battery }}%
            </span>
          </div>
          <div class="p-3 rounded-lg transition-colors" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-xs block mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">X</span>
            <span class="text-sm font-mono" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
              {{ selectedRobot.position?.world_x?.toFixed(2) || '-' }}
            </span>
          </div>
          <div class="p-3 rounded-lg transition-colors" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-xs block mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">Y</span>
            <span class="text-sm font-mono" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
              {{ selectedRobot.position?.world_y?.toFixed(2) || '-' }}
            </span>
          </div>
          <div class="col-span-2 p-3 rounded-lg transition-colors" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-xs block mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('fleet.currentTask') }}</span>
            <span class="text-sm font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
              {{ selectedRobot.current_task || t('common.none') }}
            </span>
          </div>
          <div class="col-span-2 p-3 rounded-lg transition-colors" :class="theme === 'dark' ? 'bg-industrial-700/50' : 'bg-gray-50'">
            <span class="text-xs block mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('fleet.lastUpdate') }}</span>
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              {{ selectedRobot.last_update || '-' }}
            </span>
          </div>
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
import { Bot, MapPin, Battery, ListTodo, StopCircle, RefreshCw, MoreVertical, X, Undo2, Navigation, UserMinus } from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const { language, theme } = storeToRefs(settingsStore)
const { t } = settingsStore

const filter = ref('all')
const searchQuery = ref('')
const selectedRobot = ref(null)

const filterOptions = computed(() => [
  { value: 'all', label: t('fleet.filterAll') },
  { value: 'online', label: t('fleet.filterOnline') },
  { value: 'working', label: t('fleet.filterWorking') },
  { value: 'idle', label: t('fleet.filterIdle') },
  { value: 'offline', label: t('fleet.filterOffline') }
])

const robots = computed(() => Object.entries(fleetStore.robots).map(([id, data]) => ({ id, ...data })))

const filteredRobots = computed(() => {
  return robots.value.filter(robot => {
    // offline 状态判断：status 为 offline 或 online 为 false
    const isOffline = !robot.online || robot.status === 'offline'
    let matchFilter = true
    if (filter.value === 'online') matchFilter = robot.online
    else if (filter.value === 'offline') matchFilter = isOffline
    else if (filter.value === 'working') matchFilter = robot.status === 'working' || robot.status === 'moving'
    else if (filter.value === 'idle') matchFilter = robot.online && (robot.status === 'idle' || robot.status === 'arrived')
    const matchSearch = !searchQuery.value || robot.id.toLowerCase().includes(searchQuery.value.toLowerCase())
    return matchFilter && matchSearch
  })
})

function statusText(status) {
  const map = { 
    online: t('common.online'), 
    offline: t('common.offline'), 
    working: t('common.working'), 
    idle: t('common.idle'), 
    charging: t('common.charging'),
    moving: t('common.moving'),
    arrived: t('common.arrived'),
    failed: t('tasks.failed'),
    unknown: t('common.unknown')
  }
  return map[status] || status
}

function statusClass(status) {
  const classes = {
    online: 'bg-cyber-green/20 text-cyber-green',
    offline: 'bg-cyber-red/20 text-cyber-red',
    working: 'bg-cyber-blue/20 text-cyber-blue',
    idle: 'bg-gray-500/20 text-gray-400',
    charging: 'bg-cyber-yellow/20 text-cyber-yellow',
    moving: 'bg-cyber-blue/20 text-cyber-blue',
    arrived: 'bg-cyber-purple/20 text-cyber-purple',
    failed: 'bg-cyber-red/20 text-cyber-red',
    unknown: 'bg-gray-500/20 text-gray-400'
  }
  return classes[status] || 'bg-gray-500/20 text-gray-400'
}

function statusDotClass(status) {
  const classes = {
    online: 'bg-cyber-green',
    offline: 'bg-cyber-red',
    working: 'bg-cyber-blue',
    idle: 'bg-gray-500',
    charging: 'bg-cyber-yellow',
    moving: 'bg-cyber-blue',
    arrived: 'bg-cyber-purple',
    failed: 'bg-cyber-red',
    unknown: 'bg-gray-500'
  }
  return classes[status] || 'bg-gray-500'
}

function statusTextClass(status) {
  const classes = {
    online: 'text-cyber-green',
    offline: 'text-cyber-red',
    working: 'text-cyber-blue',
    idle: 'text-gray-400',
    charging: 'text-cyber-yellow',
    moving: 'text-cyber-blue',
    arrived: 'text-cyber-purple',
    failed: 'text-cyber-red',
    unknown: 'text-gray-400'
  }
  return classes[status] || 'text-gray-400'
}

function batteryClass(battery) {
  if (battery > 60) return 'bg-cyber-green'
  if (battery > 20) return 'bg-cyber-orange'
  return 'bg-cyber-red'
}

function batteryTextClass(battery) {
  if (battery > 60) return 'text-cyber-green'
  if (battery > 20) return 'text-cyber-orange'
  return 'text-cyber-red'
}

/** 航线上：结合 nav_status 区分运动中 / 停在线上 */
function segmentMotionHint(robot) {
  const nav = robot.nav_status || ''
  if (nav === 'failed') return t('tasks.failed')
  return t('fleet.segmentMoving')
}

function showRobotDetail(robot) { selectedRobot.value = robot }

function assignTask(robotId) {
  // 简单跳转到任务页，并写入预选机器人（避免现在做一套复杂弹窗）
  localStorage.setItem('prefill_robot_id', robotId)
  window.location.hash = '#/tasks'
}

async function recallRobot(robotId) {
  // ROS2 版本后端目前没有“召回”REST接口；先走 WebSocket 指令通道（如果后端实现了会生效）
  const ok = fleetStore.sendCommand('recall_robot', { robot_id: robotId })
  if (!ok) fleetStore.addLog('warning', 'WebSocket未连接，无法召回')
}

async function stopRobot(robotId) {
  try {
    const res = await fetch(`/api/robots/${encodeURIComponent(robotId)}/stop`, { method: 'POST' })
    if (!res.ok) {
      const err = await res.json().catch(() => ({}))
      throw new Error(err.detail || `stop failed: ${res.status}`)
    }
    fleetStore.addLog('warning', `Stop ${robotId}`)
  } catch (e) {
    fleetStore.addLog('error', `Stop ${robotId} failed`)
    console.error(e)
  }
}

async function removeRobot(robotId) {
  if (!confirm(t('fleet.removeRobotConfirm', { id: robotId }))) return
  try {
    const res = await fetch(`/api/robots/${encodeURIComponent(robotId)}/remove`, { method: 'POST' })
    const data = await res.json()
    if (!res.ok) {
      throw new Error(data.detail || `remove failed: ${res.status}`)
    }
    fleetStore.addLog('warn', t('fleet.removeRobotSuccess', { id: robotId }))
  } catch (e) {
    fleetStore.addLog('error', t('fleet.removeRobotFailed', { id: robotId }) + ': ' + e.message)
    console.error(e)
  }
}

async function emergencyStopAll() {
  try {
    const res = await fetch('/api/emergency_stop', { method: 'POST' })
    if (!res.ok) {
      const err = await res.json().catch(() => ({}))
      throw new Error(err.detail || `emergency stop failed: ${res.status}`)
    }
    fleetStore.addLog('error', 'Emergency stop all')
  } catch (e) {
    fleetStore.addLog('error', 'Emergency stop failed')
    console.error(e)
  }
}

async function refreshFleet() {
  try {
    await fleetStore.refreshRobots()
    fleetStore.addLog('info', 'Fleet refreshed')
  } catch (e) {
    fleetStore.addLog('error', 'Refresh failed')
    console.error(e)
  }
}
</script>
