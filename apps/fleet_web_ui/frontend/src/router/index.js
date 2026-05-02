import { createRouter, createWebHistory } from 'vue-router'

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

const router = createRouter({
  history: createWebHistory(),
  routes
})

export default router
