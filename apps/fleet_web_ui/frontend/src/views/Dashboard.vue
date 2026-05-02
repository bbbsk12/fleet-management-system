<template>
  <div class="space-y-4 sm:space-y-6">
    <div class="flex items-center justify-between flex-wrap gap-2 sm:gap-4">
      <div>
        <h1 class="text-xl sm:text-2xl font-bold tracking-wide" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('dashboard.title') }}</h1>
        <p class="text-xs sm:text-sm mt-0.5 sm:mt-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('dashboard.subtitle') }}</p>
      </div>
      <div class="flex items-center gap-2">
        <div class="w-2 h-2 rounded-full" :class="rosConnected ? 'bg-cyber-green' : 'bg-cyber-red'" />
        <span class="text-xs sm:text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
          {{ rosConnected ? t('dashboard.realtimeData') : t('common.offline') }}
        </span>
      </div>
    </div>

    <div class="grid grid-cols-2 lg:grid-cols-4 gap-3 sm:gap-4">
      <DataCard 
        :title="t('dashboard.robots')" 
        :value="totalRobots" 
        :icon="Bot"
        color="blue"
      />
      <DataCard 
        :title="t('dashboard.onlineRobots')" 
        :value="onlineRobots" 
        :icon="Wifi"
        color="green"
      />
      <DataCard 
        :title="t('dashboard.activeTasks')" 
        :value="activeTasks" 
        :icon="Activity"
        color="orange"
      />
      <DataCard 
        :title="t('dashboard.todayTask')" 
        :value="completedTasks" 
        :icon="CheckCircle"
        color="purple"
      />
    </div>

    <div class="grid grid-cols-1 lg:grid-cols-3 gap-4 sm:gap-6">
      <div class="lg:col-span-2 data-card rounded-lg sm:rounded-xl p-4 sm:p-6 neon-border">
        <div class="flex items-center justify-between mb-4 sm:mb-6">
          <h2 class="text-base sm:text-lg font-semibold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('dashboard.fleetStatus') }}</h2>
          <router-link to="/fleet" class="btn-cyber px-3 py-1.5 rounded-lg text-sm">
            {{ t('dashboard.viewAll') }}
          </router-link>
        </div>
        
        <div class="overflow-x-auto">
          <table class="w-full">
            <thead>
              <tr :class="theme === 'dark' ? 'border-industrial-600' : 'border-gray-200'" style="border-bottom-width: 1px; border-bottom-style: solid;">
                <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">ID</th>
                <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('common.status') }}</th>
                <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('fleet.battery') }}</th>
                <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('fleet.currentTask') }}</th>
              </tr>
            </thead>
            <tbody>
              <tr 
                v-for="robot in recentRobots" 
                :key="robot.id" 
                class="transition-colors"
                :class="theme === 'dark' ? 'border-industrial-700/50 hover:bg-industrial-700/30' : 'border-gray-100 hover:bg-gray-50'"
                style="border-bottom-width: 1px; border-bottom-style: solid;"
              >
                <td class="py-3 px-4">
                  <span class="font-medium text-cyber-blue">{{ robot.id }}</span>
                </td>
                <td class="py-3 px-4">
                  <span 
                    class="inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium"
                    :class="statusClass(robot.status)"
                  >
                    <span class="w-1.5 h-1.5 rounded-full" :class="statusDotClass(robot.status)" />
                    {{ statusText(robot.status) }}
                  </span>
                </td>
                <td class="py-3 px-4">
                  <div class="flex items-center gap-2">
                    <div class="w-16 h-2 rounded-full overflow-hidden" :class="theme === 'dark' ? 'bg-industrial-700' : 'bg-gray-200'">
                      <div 
                        class="h-full rounded-full transition-all duration-300"
                        :class="batteryClass(robot.battery)"
                        :style="{ width: robot.battery + '%' }"
                      />
                    </div>
                    <span class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ robot.battery }}%</span>
                  </div>
                </td>
                <td class="py-3 px-4 text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
                  {{ robot.current_task || '-' }}
                </td>
              </tr>
              <tr v-if="recentRobots.length === 0">
                <td colspan="4" class="py-8 text-center" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
                  {{ t('fleet.noRobots') }}
                </td>
              </tr>
            </tbody>
          </table>
        </div>
      </div>

      <div class="data-card rounded-lg sm:rounded-xl p-4 sm:p-6 neon-border">
        <div class="flex items-center justify-between mb-4 sm:mb-6">
          <h2 class="text-base sm:text-lg font-semibold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('dashboard.taskQueue') }}</h2>
          <router-link to="/tasks" class="text-sm text-cyber-blue hover:text-cyber-blue/80 transition-colors">
            {{ t('dashboard.manageTasks') }}
          </router-link>
        </div>
        
        <div class="space-y-3">
          <div 
            v-for="task in recentTasks" 
            :key="task.id"
            class="p-3 rounded-lg transition-colors"
            :class="theme === 'dark' ? 'bg-industrial-700/50 hover:bg-industrial-700/70' : 'bg-gray-50 hover:bg-gray-100'"
          >
            <div class="flex items-center justify-between mb-2">
              <span class="font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ task.id }}</span>
              <span 
                class="px-2 py-0.5 rounded text-xs font-medium"
                :class="taskStatusClass(task.status)"
              >
                {{ taskStatusText(task.status) }}
              </span>
            </div>
            <div class="flex items-center justify-between text-sm">
              <span :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('dashboard.target') }}: {{ task.waypoint_id || '-' }}</span>
              <span v-if="task.robot_id" class="text-cyber-blue">{{ task.robot_id }}</span>
            </div>
          </div>
          
          <div v-if="recentTasks.length === 0" class="py-8 text-center" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
            <ListTodo class="w-8 h-8 mx-auto mb-2 opacity-50" />
            <p class="text-sm">{{ t('tasks.noTasks') }}</p>
          </div>
        </div>
      </div>
    </div>

    <div class="data-card rounded-lg sm:rounded-xl p-4 sm:p-6 neon-border">
      <div class="flex items-center justify-between mb-4 sm:mb-6">
          <h2 class="text-base sm:text-lg font-semibold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('dashboard.recentLogs') }}</h2>
          <router-link to="/logs" class="text-sm text-cyber-blue hover:text-cyber-blue/80 transition-colors">
            {{ t('dashboard.viewAll') }}
          </router-link>
        </div>
        
        <div class="space-y-2 max-h-64 overflow-y-auto">
          <div 
            v-for="(log, index) in recentLogs" 
            :key="index"
            class="flex items-start gap-3 p-2 rounded transition-colors"
            :class="theme === 'dark' ? 'hover:bg-industrial-700/30' : 'hover:bg-gray-50'"
          >
            <span class="text-xs font-mono shrink-0" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ log.time }}</span>
            <span 
              class="text-xs px-1.5 py-0.5 rounded font-medium shrink-0"
              :class="logLevelClass(log.level)"
            >
              {{ log.level.toUpperCase() }}
            </span>
            <span class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ log.message }}</span>
          </div>
          
          <div v-if="recentLogs.length === 0" class="py-8 text-center" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
            <FileText class="w-8 h-8 mx-auto mb-2 opacity-50" />
            <p class="text-sm">{{ t('logs.noLogs') }}</p>
          </div>
        </div>
    </div>
  </div>
</template>

<script setup>
import { computed } from 'vue'
import { storeToRefs } from 'pinia'
import { useFleetStore } from '../stores/fleet'
import { useSettingsStore } from '../stores/settings'
import DataCard from '../components/DataCard.vue'
import { Bot, Wifi, Activity, CheckCircle, ListTodo, FileText } from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const { language, theme } = storeToRefs(settingsStore)
const { t } = settingsStore

const rosConnected = computed(() => fleetStore.rosConnected)
const totalRobots = computed(() => fleetStore.totalRobots)
const onlineRobots = computed(() => fleetStore.onlineRobots)
const activeTasks = computed(() => fleetStore.activeTasks)

const completedTasks = computed(() => fleetStore.tasks.filter(t => t.status === 'completed').length)

const recentRobots = computed(() => {
  return Object.entries(fleetStore.robots)
    .slice(0, 5)
    .map(([id, data]) => ({
      id,
      ...data
    }))
})

const recentTasks = computed(() => {
  return fleetStore.tasks.slice(0, 5)
})

const recentLogs = computed(() => {
  return fleetStore.logs.slice(0, 8)
})

function statusText(status) {
  const map = {
    online: t('common.online'),
    offline: t('common.offline'),
    working: t('dashboard.working'),
    idle: t('common.idle'),
    charging: t('common.charging')
  }
  return map[status] || status
}

function statusClass(status) {
  const classes = {
    online: 'bg-cyber-green/20 text-cyber-green',
    offline: 'bg-cyber-red/20 text-cyber-red',
    working: 'bg-cyber-blue/20 text-cyber-blue',
    idle: 'bg-gray-500/20 text-gray-400',
    charging: 'bg-cyber-yellow/20 text-cyber-yellow'
  }
  return classes[status] || 'bg-gray-500/20 text-gray-400'
}

function statusDotClass(status) {
  const classes = {
    online: 'bg-cyber-green',
    offline: 'bg-cyber-red',
    working: 'bg-cyber-blue',
    idle: 'bg-gray-500',
    charging: 'bg-cyber-yellow'
  }
  return classes[status] || 'bg-gray-500'
}

function batteryClass(battery) {
  if (battery > 60) return 'bg-cyber-green'
  if (battery > 20) return 'bg-cyber-orange'
  return 'bg-cyber-red'
}

function taskStatusText(status) {
  const map = {
    pending: t('tasks.pending'),
    assigned: t('dashboard.assigned'),
    running: t('tasks.running'),
    completed: t('tasks.completed'),
    failed: t('tasks.failed')
  }
  return map[status] || status
}

function taskStatusClass(status) {
  const classes = {
    pending: 'bg-gray-500/20 text-gray-400',
    assigned: 'bg-cyber-blue/20 text-cyber-blue',
    running: 'bg-cyber-green/20 text-cyber-green',
    completed: 'bg-cyber-purple/20 text-cyber-purple',
    failed: 'bg-cyber-red/20 text-cyber-red'
  }
  return classes[status] || 'bg-gray-500/20 text-gray-400'
}

function logLevelClass(level) {
  const classes = {
    info: 'bg-cyber-blue/20 text-cyber-blue',
    warn: 'bg-cyber-orange/20 text-cyber-orange',
    error: 'bg-cyber-red/20 text-cyber-red',
    debug: 'bg-gray-500/20 text-gray-400'
  }
  return classes[level] || 'bg-gray-500/20 text-gray-400'
}

</script>

<style scoped>
.btn-cyber {
  @apply relative inline-flex items-center justify-center gap-2;
  @apply bg-gradient-to-r from-cyber-blue/10 to-cyber-blue/5;
  @apply border border-cyber-blue/40 text-cyber-blue;
  @apply transition-all duration-200;
  @apply hover:from-cyber-blue/20 hover:to-cyber-blue/10;
  @apply hover:border-cyber-blue hover:shadow-glow;
}
</style>
