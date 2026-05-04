<template>
  <!-- 根容器：全屏 flex 布局，根据主题切换暗色模式 -->
  <div class="flex h-screen grid-bg" :class="theme === 'dark' ? 'dark' : ''">
    <!-- 移动端遮罩层 -->
    <div
      v-if="isMobile && sidebarOpen"
      class="fixed inset-0 bg-black/50 z-40 transition-opacity duration-300"
      @click="sidebarOpen = false"
    />

    <!-- 侧边栏：包含导航菜单与系统状态 -->
    <aside
      class="flex flex-col transition-all duration-300"
      :class="[
        theme === 'dark' ? 'bg-industrial-800 border-industrial-600' : 'bg-white border-gray-200',
        isMobile
          ? 'fixed top-0 left-0 h-full z-50 w-72 -translate-x-full'
          : 'w-64'
      ]"
      :style="{
        borderRightWidth: '1px',
        borderRightStyle: 'solid',
        transform: isMobile
          ? (sidebarOpen ? 'translateX(0)' : 'translateX(-100%)')
          : 'none'
      }"
    >
      <!-- 侧边栏头部：Logo 与应用名称 -->
      <div class="h-16 flex items-center justify-between px-5" :class="theme === 'dark' ? 'border-industrial-600' : 'border-gray-200'" :style="{ borderBottomWidth: '1px', borderBottomStyle: 'solid' }">
        <div class="flex items-center gap-3">
          <div class="p-2 rounded-lg" :class="theme === 'dark' ? 'bg-cyber-blue/10' : 'bg-cyber-blue/10'">
            <Bot class="w-6 h-6 text-cyber-blue" />
          </div>
          <div>
            <h1 class="text-lg font-bold tracking-wider" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">FleetOS</h1>
            <p class="text-[10px] tracking-widest" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('app.subtitle') }}</p>
          </div>
        </div>
        <!-- 移动端侧边栏关闭按钮 -->
        <button
          v-if="isMobile"
          @click="sidebarOpen = false"
          class="p-2 rounded-lg"
          :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'"
        >
          <X class="w-5 h-5" />
        </button>
      </div>

      <!-- 导航菜单区域 -->
      <nav class="flex-1 py-6 px-3 overflow-y-auto">
        <!-- 监控中心分组 -->
        <div class="mb-4 px-4">
          <span class="text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('sidebar.monitorCenter') }}</span>
        </div>

        <!-- 主路由导航项 -->
        <div class="space-y-1">
          <router-link
            v-for="route in mainRoutes"
            :key="route.path"
            :to="route.path"
            class="nav-item"
            :class="{ active: $route.path === route.path }"
            @click="isMobile && (sidebarOpen = false)"
          >
            <component :is="route.meta.iconComponent" class="w-5 h-5" />
            <span class="text-sm font-medium">{{ t(route.meta.titleKey) }}</span>
            <div v-if="$route.path === route.path" class="ml-auto w-2 h-2 rounded-full bg-cyber-blue" />
          </router-link>
        </div>

        <!-- 系统分组 -->
        <div class="mt-6 mb-4 px-4">
          <span class="text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('sidebar.system') }}</span>
        </div>

        <!-- 系统导航项 -->
        <div class="space-y-1">
          <router-link to="/logs" class="nav-item" :class="{ active: $route.path === '/logs' }" @click="isMobile && (sidebarOpen = false)">
            <FileText class="w-5 h-5" />
            <span class="text-sm font-medium">{{ t('sidebar.logs') }}</span>
          </router-link>
          <router-link to="/settings" class="nav-item" :class="{ active: $route.path === '/settings' }" @click="isMobile && (sidebarOpen = false)">
            <Settings class="w-5 h-5" />
            <span class="text-sm font-medium">{{ t('sidebar.settings') }}</span>
          </router-link>
        </div>
      </nav>

      <!-- 底部系统状态面板 -->
      <div class="p-4" :class="theme === 'dark' ? 'border-industrial-600' : 'border-gray-200'" :style="{ borderTopWidth: '1px', borderTopStyle: 'solid' }">
        <div class="data-card rounded-lg p-4">
          <div class="flex items-center gap-2 mb-3">
            <Activity class="w-4 h-4" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'" />
            <span class="text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('sidebar.systemStatus') }}</span>
          </div>
          <div class="space-y-2 text-xs sm:text-sm">
            <div class="flex justify-between items-center">
              <span :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('sidebar.rosConnection') }}</span>
              <div class="flex items-center gap-2">
                <span :class="rosConnected ? 'text-cyber-green' : 'text-cyber-red'" class="font-medium">
                  {{ rosConnected ? t('sidebar.connected') : t('sidebar.notConnected') }}
                </span>
                <div class="w-2 h-2 rounded-full" :class="rosConnected ? 'bg-cyber-green' : 'bg-cyber-red'" />
              </div>
            </div>
            <div class="flex justify-between items-center">
              <span :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('sidebar.onlineDevices') }}</span>
              <span class="text-cyber-blue font-medium">{{ onlineRobots }}/{{ totalRobots }}</span>
            </div>
          </div>
        </div>
      </div>
    </aside>

    <!-- 主内容区域 -->
    <main class="flex-1 flex flex-col overflow-hidden min-w-0">
      <!-- 顶部导航栏 -->
      <header class="h-14 flex items-center justify-between px-3 sm:px-4 transition-colors duration-300 shrink-0" :class="theme === 'dark' ? 'bg-industrial-800 border-industrial-600' : 'bg-white border-gray-200'" :style="{ borderBottomWidth: '1px', borderBottomStyle: 'solid' }">
        <!-- 移动端菜单切换按钮 -->
        <button
          v-if="isMobile"
          @click="sidebarOpen = !sidebarOpen"
          class="p-2 rounded-lg hover:bg-gray-100 dark:hover:bg-industrial-700"
          :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'"
        >
          <Menu class="w-5 h-5" />
        </button>
        <div class="flex-1" />
        <!-- 右侧工具栏：连接状态指示与主题/语言切换 -->
        <div class="flex items-center gap-2 sm:gap-4">
          <!-- ROS 连接状态指示 -->
          <div class="flex items-center gap-2">
            <div class="w-2 h-2 rounded-full" :class="rosConnected ? 'bg-cyber-green' : 'bg-cyber-red'" />
            <span class="text-sm hidden sm:inline" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ rosConnected ? t('sidebar.liveMonitor') : t('sidebar.notConnected') }}</span>
          </div>

          <!-- 主题切换与语言切换按钮 -->
          <div class="flex items-center gap-1 sm:gap-2">
            <button
              @click="toggleTheme"
              class="p-2 rounded-lg transition-colors"
              :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'"
              :title="theme === 'dark' ? t('sidebar.switchToLight') : t('sidebar.switchToDark')"
            >
              <Sun v-if="theme === 'dark'" class="w-5 h-5" />
              <Moon v-else class="w-5 h-5" />
            </button>

            <button
              @click="toggleLanguage"
              class="px-2 sm:px-3 py-1.5 rounded-lg text-sm font-medium transition-colors"
              :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-300' : 'hover:bg-gray-100 text-gray-600'"
            >
              {{ language === 'zh' ? 'EN' : '中文' }}
            </button>
          </div>
        </div>
      </header>

      <!-- 页面路由视图：带过渡动画 -->
      <div class="flex-1 overflow-auto p-3 sm:p-6" :class="theme === 'dark' ? 'bg-industrial-900' : 'bg-gray-50'">
        <router-view v-slot="{ Component }">
          <transition name="fade" mode="out-in">
            <component :is="Component" />
          </transition>
        </router-view>
      </div>
    </main>
  </div>
</template>

<script setup>
// 基础 Vue API 导入
import { ref, computed, onMounted, onUnmounted, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useFleetStore } from './stores/fleet'
import { useSettingsStore } from './stores/settings'
import { useRouter, useRoute } from 'vue-router'

// Lucide 图标导入
import { Bot, LayoutDashboard, Map, ListTodo, Settings, FileText, Activity, Menu, Sun, Moon, X } from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const router = useRouter()
const route = useRoute()

// 侧边栏与移动端状态
const sidebarOpen = ref(false)
const isMobile = ref(false)

// 从 store 中解构响应式状态与方法
const { language, theme } = storeToRefs(settingsStore)
const { setLanguage, setTheme, t } = settingsStore

// 计算主路由信息：为各路由绑定对应图标与标题键
const mainRoutes = computed(() => {
  return router.options.routes.filter(r =>
    ['/', '/fleet', '/tasks', '/map'].includes(r.path)
  ).map(r => {
    const iconMap = {
      '/': LayoutDashboard,
      '/fleet': Bot,
      '/tasks': ListTodo,
      '/map': Map
    }
    const titleKeyMap = {
      '/': 'sidebar.dashboard',
      '/fleet': 'sidebar.fleet',
      '/tasks': 'sidebar.tasks',
      '/map': 'sidebar.map'
    }
    return {
      ...r,
      meta: {
        ...r.meta,
        iconComponent: iconMap[r.path] || LayoutDashboard,
        titleKey: titleKeyMap[r.path] || 'common.status'
      }
    }
  })
})

// ROS 连接状态与设备统计
const rosConnected = computed(() => fleetStore.rosConnected)
const onlineRobots = computed(() => fleetStore.onlineRobots)
const totalRobots = computed(() => fleetStore.totalRobots)

// 切换明暗主题
function toggleTheme() {
  setTheme(theme.value === 'dark' ? 'light' : 'dark')
}

// 切换中英文语言
function toggleLanguage() {
  setLanguage(language.value === 'zh' ? 'en' : 'zh')
}

// 检测是否为移动端设备，自动关闭侧边栏
function checkMobile() {
  isMobile.value = window.innerWidth < 768
  if (!isMobile.value) {
    sidebarOpen.value = false
  }
}

// 路由变化时自动关闭移动端侧边栏
watch(route, () => {
  if (isMobile.value) {
    sidebarOpen.value = false
  }
})

onMounted(() => {
  fleetStore.connectWebSocket()
  checkMobile()
  window.addEventListener('resize', checkMobile)
})

onUnmounted(() => {
  window.removeEventListener('resize', checkMobile)
})
</script>

<!-- 导航项与过渡动画样式 -->
<style scoped>
.nav-item {
  @apply w-full flex items-center px-4 py-3.5 rounded-lg transition-all duration-200;
}

.nav-item.light {
  @apply text-gray-600 hover:bg-gray-100 hover:text-gray-900;
}

.nav-item.dark {
  @apply text-gray-400 hover:bg-industrial-700 hover:text-white;
}

.nav-item.active {
  @apply bg-gradient-to-r from-cyber-blue/20 to-transparent border-l-2 border-cyber-blue text-cyber-blue;
}

/* 页面切换淡入淡出动画 */
.fade-enter-active,
.fade-leave-active {
  transition: opacity 0.2s ease;
}

.fade-enter-from,
.fade-leave-to {
  opacity: 0;
}
</style>
