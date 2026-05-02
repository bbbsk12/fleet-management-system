import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

export const useFleetStore = defineStore('fleet', () => {
  // 状态
  const robots = ref({})
  const tasks = ref([])
  const trafficMap = ref(null)
  const waypoints = ref([])
  const mapData = ref(null)
  const logs = ref([])
  const rosConnected = ref(false)
  const ws = ref(null)
  
  function getSavedSettings() {
    try {
      const raw = localStorage.getItem('fleet_settings')
      return raw ? JSON.parse(raw) : {}
    } catch {
      return {}
    }
  }

  // 计算属性
  const totalRobots = computed(() => Object.keys(robots.value).length)
  const onlineRobots = computed(() => {
    return Object.values(robots.value).filter(r => r.online).length
  })
  const activeTasks = computed(() => {
    return tasks.value.filter(t => ['pending', 'assigned', 'running', 'in_progress'].includes(t.status)).length
  })
  
  // WebSocket连接
  function connectWebSocket() {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:'
    const saved = getSavedSettings()
    const wsPort = Number(saved.ws_port) || window.location.port
    const host = window.location.hostname
    const wsUrl = `${protocol}//${host}${wsPort ? `:${wsPort}` : ''}/ws`
    
    ws.value = new WebSocket(wsUrl)
    
    ws.value.onopen = () => {
      rosConnected.value = true
      addLog('info', 'WebSocket连接成功')
    }
    
    ws.value.onclose = () => {
      rosConnected.value = false
      addLog('warning', 'WebSocket连接断开，5秒后重连...')
      setTimeout(connectWebSocket, 5000)
    }
    
    ws.value.onerror = (error) => {
      addLog('error', 'WebSocket错误')
    }
    
    ws.value.onmessage = (event) => {
      const data = JSON.parse(event.data)
      handleMessage(data)
    }
  }
  
  // 处理消息
  function handleMessage(data) {
    switch (data.type) {
      case 'init':
        // 初始化数据
        if (data.payload.robots) {
          robots.value = data.payload.robots
        }
        if (data.payload.tasks) {
          tasks.value = data.payload.tasks
        }
        if (data.payload.waypoints) {
          waypoints.value = data.payload.waypoints
        }
        if (data.payload.map_data) {
          mapData.value = data.payload.map_data
        }
        if (typeof data.payload.ros_connected === 'boolean') {
          rosConnected.value = data.payload.ros_connected
        }
        break
      case 'robot_status':
        robots.value[data.robot_id] = data.payload
        break
      case 'fleet_status':
        // 合并而非覆盖：新消息包含所有机器人（含 offline，后端已合并）
        // 直接整体替换确保响应式触发
        robots.value = { ...data.payload }
        break
      case 'task_update':
        updateTask(data.payload)
        break
      case 'task_created':
        tasks.value.push(data.task)
        break
      case 'log':
        addLog(data.level, data.message)
        break
      case 'robot_removed':
        // 机器人被移除出队，从本地状态中删除
        if (data.payload && data.payload.robot_id) {
          delete robots.value[data.payload.robot_id]
          robots.value = { ...robots.value }  // 触发响应式
        }
        break
    }
  }
  
  // 更新任务
  function updateTask(task) {
    const index = tasks.value.findIndex(t => t.id === task.id)
    if (index >= 0) {
      tasks.value[index] = task
    } else {
      tasks.value.push(task)
    }
  }
  
  // 添加日志
  function addLog(level, message) {
    logs.value.unshift({
      time: new Date().toLocaleTimeString(),
      level,
      message
    })
    // 保留最近500条日志
    if (logs.value.length > 500) {
      logs.value.pop()
    }
  }
  
  // 发送命令
  function sendCommand(type, payload) {
    if (ws.value && ws.value.readyState === WebSocket.OPEN) {
      ws.value.send(JSON.stringify({ type, payload }))
      return true
    }
    return false
  }

  async function refreshRobots() {
    const res = await fetch('/api/robots')
    if (!res.ok) throw new Error(`Failed to refresh robots: ${res.status}`)
    const data = await res.json()
    if (data?.robots) robots.value = data.robots
  }
  
  // 提交任务
  function submitTask(robotId, waypointId, priority = 0, taskType = 1, siteCode = 0) {
    return sendCommand('submit_task', { robot_id: robotId, waypoint_id: waypointId, priority, task_type: taskType, site_code: siteCode })
  }
  
  // 取消任务
  function cancelTask(taskId) {
    return sendCommand('cancel_task', { task_id: taskId })
  }
  
  // 紧急停止
  function emergencyStop(robotId = null) {
    return sendCommand('emergency_stop', { robot_id: robotId })
  }
  
  // 加载交通图
  function loadTrafficMap(mapPath) {
    return sendCommand('load_map', { map_path: mapPath })
  }
  
  return {
    // 状态
    robots,
    tasks,
    trafficMap,
    waypoints,
    mapData,
    logs,
    rosConnected,
    ws,
    // 计算属性
    totalRobots,
    onlineRobots,
    activeTasks,
    // 方法
    connectWebSocket,
    sendCommand,
    refreshRobots,
    submitTask,
    cancelTask,
    emergencyStop,
    loadTrafficMap,
    addLog
  }
})
