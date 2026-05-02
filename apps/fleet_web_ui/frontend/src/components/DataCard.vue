<template>
  <component
    :is="onClick ? 'button' : 'div'"
    @click="onClick"
    class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 md:p-5 neon-border transition-all duration-200 ease-out"
    :class="{ 'cursor-pointer': onClick }"
    :style="{ minHeight: '44px' }"
  >
    <div class="flex items-start justify-between mb-2 sm:mb-3">
      <span class="text-xs sm:text-sm uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
        {{ title }}
      </span>
      <span v-if="icon" :class="colorClasses[color].text" class="transition-transform duration-200">
        <component :is="icon" class="w-5 h-5 sm:w-6 sm:h-6" />
      </span>
    </div>
    
    <div class="flex items-baseline gap-1 sm:gap-2">
      <span :class="['text-xl sm:text-2xl md:text-3xl font-bold', colorClasses[color].text]">
        {{ value }}
      </span>
      <span v-if="unit" class="text-xs sm:text-sm font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
        {{ unit }}
      </span>
      <span v-if="trend" class="ml-auto text-sm" :class="trendColors[trend]">
        {{ trendLabels[trend] }}
      </span>
    </div>
  </component>
</template>

<script setup>
import { storeToRefs } from 'pinia'
import { useSettingsStore } from '../stores/settings'

const props = defineProps({
  title: { type: String, required: true },
  value: { type: [String, Number], required: true },
  unit: { type: String, default: '' },
  icon: { type: Object, default: null },
  trend: { type: String, default: '' },
  color: { type: String, default: 'blue' },
  onClick: { type: Function, default: null }
})

const settingsStore = useSettingsStore()
const { theme } = storeToRefs(settingsStore)

const colorClasses = {
  blue: { text: 'text-cyber-blue' },
  green: { text: 'text-cyber-green' },
  orange: { text: 'text-cyber-orange' },
  red: { text: 'text-cyber-red' },
  purple: { text: 'text-cyber-purple' },
  yellow: { text: 'text-cyber-yellow' }
}

const trendColors = {
  up: 'text-cyber-green',
  down: 'text-cyber-red',
  stable: 'text-gray-400'
}

const trendLabels = {
  up: '↑',
  down: '↓',
  stable: '→'
}
</script>
