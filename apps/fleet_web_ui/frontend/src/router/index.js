// ============================================================================
//  路由配置 —— router/index.js
//  功能：定义前端路由表，配置页面路径与对应组件的懒加载关系
// ============================================================================

import { createRouter, createWebHistory } from 'vue-router'

// ---- 路由表定义 ----
// 所有页面均采用懒加载方式，按需加载对应视图组件
const routes = [
  {
    path: '/',
    name: 'Dashboard',
    component: () => import('../views/Dashboard.vue'),
    meta: { title: '控制中心', icon: '📊' }
  },
  {
    path: '/fleet',
    name: 'Fleet',
    component: () => import('../views/FleetMonitor.vue'),
    meta: { title: '车队监控', icon: '🤖' }
  },
  {
    path: '/tasks',
    name: 'Tasks',
    component: () => import('../views/TaskDispatch.vue'),
    meta: { title: '任务调度', icon: '📋' }
  },
  {
    path: '/map',
    name: 'Map',
    component: () => import('../views/MapView.vue'),
    meta: { title: '地图视图', icon: '🗺️' }
  },
  {
    path: '/logs',
    name: 'Logs',
    component: () => import('../views/SystemLogs.vue'),
    meta: { title: '系统日志', icon: '📝' }
  },
  {
    path: '/settings',
    name: 'Settings',
    component: () => import('../views/Settings.vue'),
    meta: { title: '系统设置', icon: '⚙️' }
  }
]

// ---- 创建路由实例 ----
// 使用 HTML5 History 模式，避免 URL 中携带 # 号
const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router
