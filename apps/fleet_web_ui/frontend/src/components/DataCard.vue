<template>
  <!-- 通用数据展示卡片：支持点击交互与多种颜色主题 -->
  <component
    :is="onClick ? 'button' : 'div'"
    @click="onClick"
    class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 md:p-5 neon-border transition-all duration-200 ease-out"
    :class="{ 'cursor-pointer': onClick }"
    :style="{ minHeight: '44px' }"
  >
    <!-- 卡片头部：标题与图标 -->
    <div class="flex items-start justify-between mb-2 sm:mb-3">
      <span class="text-xs sm:text-sm uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
        {{ title }}
      </span>
      <span v-if="icon" :class="colorClasses[color].text" class="transition-transform duration-200">
        <component :is="icon" class="w-5 h-5 sm:w-6 sm:h-6" />
      </span>
    </div>

    <!-- 卡片主体：数值、单位与趋势指示 -->
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

// 组件属性定义
const props = defineProps({
  title: { type: String, required: true },       // 卡片标题
  value: { type: [String, Number], required: true }, // 显示数值
  unit: { type: String, default: '' },            // 单位
  icon: { type: Object, default: null },          // 图标组件
  trend: { type: String, default: '' },           // 趋势方向（up/down/stable）
  color: { type: String, default: 'blue' },       // 主题色
  onClick: { type: Function, default: null }      // 点击回调
})

const settingsStore = useSettingsStore()
const { theme } = storeToRefs(settingsStore)

// 颜色方案映射表
const colorClasses = {
  blue: { text: 'text-cyber-blue' },
  green: { text: 'text-cyber-green' },
  orange: { text: 'text-cyber-orange' },
  red: { text: 'text-cyber-red' },
  purple: { text: 'text-cyber-purple' },
  yellow: { text: 'text-cyber-yellow' }
}

// 趋势颜色与标签映射
const trendColors = {
  up: 'text-cyber-green',
  down: 'text-cyber-red',
  stable: 'text-gray-400'
}

const trendLabels = {
  up: '\u2191',
  down: '\u2193',
  stable: '\u2192'
}
</script>
