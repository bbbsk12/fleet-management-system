// ============================================================================
//  车队状态管理 —— stores/fleet.js
//  功能：Pinia 状态仓库，管理机器人列表、任务列表、地图数据、WebSocket 连接
//        以及所有与车队运行相关的核心业务逻辑
// ============================================================================

import { defineStore } from 'pinia'
import { ref, computed } from 'vue'

export const useFleetStore = defineStore('fleet', () => {

  // ========================================================================
  //  状态定义
  // ========================================================================

  /** 机器人字典，key 为机器人 ID，value 为机器人状态对象 */
  const robots = ref({})
  /** 任务列表 */
  const tasks = ref([])
  /** 交通地图数据 */
  const trafficMap = ref(null)
  /** 航点列表 */
  const waypoints = ref([])
  /** 地图路径/图像数据 */
  const mapData = ref(null)
  /** 系统日志记录列表 */
  const logs = ref([])
  /** ROS 连接状态标志 */
  const rosConnected = ref(false)
  /** WebSocket 连接实例 */
  const ws = ref(null)

  // ---- 调度器指标与告警 ----
  /** 调度器核心指标（来自 /fleet_manager/metrics） */
  const metrics = ref({})
  /** 调度器告警列表（来自 /fleet_manager/alerts） */
  const alerts = ref([])

  /**
   * 从 localStorage 读取已保存的系统设置
   * @returns {object} 解析后的设置对象，解析失败则返回空对象
   */
  function getSavedSettings() {
    try {
      const raw = localStorage.getItem('fleet_settings')
      return raw ? JSON.parse(raw) : {}
    } catch {
      return {}
    }
  }

  // ========================================================================
  //  计算属性
  // ========================================================================

  /** 机器人总数 */
  const totalRobots = computed(() => Object.keys(robots.value).length)
  /** 在线机器人数量 */
  const onlineRobots = computed(() => {
    return Object.values(robots.value).filter(r => r.online).length
  })
  /** 活跃任务数量（待分配、已分配、执行中、进行中） */
  const activeTasks = computed(() => {
    return tasks.value.filter(t => ['pending', 'assigned', 'running', 'in_progress'].includes(t.status)).length
  })

  // ========================================================================
  //  WebSocket 连接管理
  // ========================================================================

  /**
   * 建立 WebSocket 连接
   * 根据当前页面协议和端口自动拼接 WebSocket URL，
   * 同时支持从设置中读取自定义 WebSocket 端口
   */
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

  // ========================================================================
  //  消息处理
  // ========================================================================

  /**
   * 处理 WebSocket 收到的消息
   * 根据消息类型分发到不同的处理逻辑
   * @param {object} data - 解析后的消息对象，包含 type 和 payload 字段
   */
  function handleMessage(data) {
    switch (data.type) {
      case 'init':
        // 初始化数据：首次连接时接收全量状态数据
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
        if (data.payload.metrics) metrics.value = data.payload.metrics
        if (data.payload.alerts) alerts.value = data.payload.alerts
        break
      case 'robot_status':
        // 单个机器人状态更新
        robots.value[data.robot_id] = data.payload
        break
      case 'fleet_status':
        // 车队全量状态更新（合并而非覆盖）：
        // 新消息包含所有机器人（含 offline 状态），后端已合并
        // 直接整体替换确保触发 Vue 响应式更新
        robots.value = { ...data.payload }
        break
      case 'task_update':
        // 任务状态更新
        updateTask(data.payload)
        break
      case 'task_created':
        // 新任务创建
        tasks.value.push(data.task)
        break
      case 'log':
        // 系统日志消息
        addLog(data.level, data.message)
        break
      case 'robot_removed':
        // 机器人被移出车队，从本地状态中删除
        if (data.payload && data.payload.robot_id) {
          delete robots.value[data.payload.robot_id]
          robots.value = { ...robots.value }  // 触发响应式更新
        }
        break
      case 'alert':
        alerts.value.unshift(data.payload)
        if (alerts.value.length > 100) alerts.value.pop()
        addLog('warning', `调度告警: ${data.payload.message || JSON.stringify(data.payload)}`)
        break
      case 'metrics_update':
        metrics.value = data.payload
        break
    }
  }

  // ========================================================================
  //  任务操作
  // ========================================================================

  /**
   * 更新或添加任务
   * 根据任务 ID 查找本地任务列表，存在则更新，不存在则新增
   * @param {object} task - 任务对象
   */
  function updateTask(task) {
    const index = tasks.value.findIndex(t => t.id === task.id)
    if (index >= 0) {
      tasks.value[index] = task
    } else {
      tasks.value.push(task)
    }
  }

  // ========================================================================
  //  日志管理
  // ========================================================================

  /**
   * 添加系统日志记录
   * 新日志插入到列表头部，保留最近 500 条日志以控制内存占用
   * @param {string} level - 日志级别（info / warning / error）
   * @param {string} message - 日志内容
   */
  function addLog(level, message) {
    logs.value.unshift({
      time: new Date().toLocaleTimeString(),
      level,
      message
    })
    // 保留最近 500 条日志，超出则移除最早记录
    if (logs.value.length > 500) {
      logs.value.pop()
    }
  }

  // ========================================================================
  //  命令发送
  // ========================================================================

  /**
   * 发送命令到后端
   * 检查 WebSocket 连接状态，连接正常则发送 JSON 格式命令
   * @param {string} type - 命令类型
   * @param {object} payload - 命令参数
   * @returns {boolean} 是否成功发送
   */
  function sendCommand(type, payload) {
    if (ws.value && ws.value.readyState === WebSocket.OPEN) {
      ws.value.send(JSON.stringify({ type, payload }))
      return true
    }
    return false
  }

  /**
   * 刷新机器人列表（通过 HTTP API）
   * 从后端 /api/robots 接口拉取最新的机器人全量数据
   */
  async function refreshRobots() {
    const res = await fetch('/api/robots')
    if (!res.ok) throw new Error(`Failed to refresh robots: ${res.status}`)
    const data = await res.json()
    if (data?.robots) robots.value = data.robots
  }

  /** 从后端获取调度器核心指标 */
  async function fetchMetrics() {
    try {
      const res = await fetch('/api/metrics')
      if (res.ok) {
        const data = await res.json()
        metrics.value = data.metrics || {}
      }
    } catch (e) {
      console.error('Failed to fetch metrics:', e)
    }
  }

  /** 从后端获取调度器告警列表 */
  async function fetchAlerts() {
    try {
      const res = await fetch('/api/alerts?limit=50')
      if (res.ok) {
        const data = await res.json()
        alerts.value = data.alerts || []
      }
    } catch (e) {
      console.error('Failed to fetch alerts:', e)
    }
  }

  /**
   * 提交任务
   * @param {string} robotId - 目标机器人 ID
   * @param {string|number} waypointId - 目标航点 ID
   * @param {number} [priority=0] - 任务优先级
   * @param {number} [taskType=1] - 任务类型
   * @param {number} [siteCode=0] - 站点代码
   * @returns {boolean} 是否成功发送命令
   */
  function submitTask(robotId, waypointId, priority = 0, taskType = 1, siteCode = 0) {
    return sendCommand('submit_task', { robot_id: robotId, waypoint_id: waypointId, priority, task_type: taskType, site_code: siteCode })
  }

  /**
   * 取消任务
   * @param {string|number} taskId - 任务 ID
   * @returns {boolean} 是否成功发送命令
   */
  function cancelTask(taskId) {
    return sendCommand('cancel_task', { task_id: taskId })
  }

  /**
   * 紧急停止
   * @param {string|null} robotId - 机器人 ID，为 null 时停止所有机器人
   * @returns {boolean} 是否成功发送命令
   */
  function emergencyStop(robotId = null) {
    return sendCommand('emergency_stop', { robot_id: robotId })
  }

  /**
   * 加载交通地图
   * @param {string} mapPath - 地图文件路径
   * @returns {boolean} 是否成功发送命令
   */
  function loadTrafficMap(mapPath) {
    return sendCommand('load_map', { map_path: mapPath })
  }

  // ========================================================================
  //  导出接口
  // ========================================================================

  return {
    // ---- 状态 ----
    robots,
    tasks,
    trafficMap,
    waypoints,
    mapData,
    logs,
    rosConnected,
    ws,
    // ---- 调度器指标与告警 ----
    metrics,
    alerts,
    // ---- 计算属性 ----
    totalRobots,
    onlineRobots,
    activeTasks,
    // ---- 方法 ----
    connectWebSocket,
    sendCommand,
    refreshRobots,
    fetchMetrics,
    fetchAlerts,
    submitTask,
    cancelTask,
    emergencyStop,
    loadTrafficMap,
    addLog
  }
})
