// ============================================================================
//  应用入口 —— main.js
//  功能：Vue 3 应用初始化，挂载 Pinia 状态管理、Vue Router 路由及全局样式
// ============================================================================

import { createApp } from 'vue'
import { createPinia } from 'pinia'
import router from './router'
import App from './App.vue'
import './assets/css/main.css'

// ---- 创建 Vue 应用实例 ----
const app = createApp(App)

// ---- 注册插件 ----
// 注册 Pinia 状态管理
app.use(createPinia())
// 注册 Vue Router 路由
app.use(router)

// ---- 挂载应用到 DOM ----
app.mount('#app')
