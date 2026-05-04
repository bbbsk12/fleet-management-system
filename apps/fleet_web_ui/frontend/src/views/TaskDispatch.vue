<template>
  <div class="space-y-4 sm:space-y-6">
    <!-- 页面头部：标题与创建任务按钮 -->
    <div class="flex items-center justify-between flex-wrap gap-2 sm:gap-4">
      <div>
        <h1 class="text-xl sm:text-2xl font-bold tracking-wide" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('tasks.title') }}</h1>
        <p class="text-xs sm:text-sm mt-0.5" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('tasks.subtitle') }}</p>
      </div>
      <button
        class="px-4 py-2 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue text-sm font-medium hover:bg-cyber-blue/30 transition-all flex items-center gap-2"
        @click="showNewTaskModal = true"
      >
        <Plus class="w-4 h-4" />
        {{ t('tasks.createTask') }}
      </button>
    </div>

    <!-- 任务统计概览卡片 -->
    <div class="grid grid-cols-2 lg:grid-cols-4 gap-3 sm:gap-4">
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-orange/10">
            <Clock class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-orange" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.pending') }}</p>
            <p class="text-xl sm:text-2xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ taskStats.pending }}</p>
          </div>
        </div>
      </div>
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-green/10">
            <Loader2 class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-green" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.running') }}</p>
            <p class="text-xl sm:text-2xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ taskStats.running }}</p>
          </div>
        </div>
      </div>
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-purple/10">
            <CheckCircle class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-purple" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.completed') }}</p>
            <p class="text-xl sm:text-2xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ taskStats.completed }}</p>
          </div>
        </div>
      </div>
      <div class="data-card rounded-lg sm:rounded-xl p-3 sm:p-4 neon-border">
        <div class="flex items-center gap-2 sm:gap-3">
          <div class="p-2 rounded-lg bg-cyber-red/10">
            <XCircle class="w-5 h-5 sm:w-6 sm:h-6 text-cyber-red" />
          </div>
          <div>
            <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.failed') }}</p>
            <p class="text-xl sm:text-2xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ taskStats.failed }}</p>
          </div>
        </div>
      </div>
    </div>

    <!-- 任务状态切换标签 -->
    <div class="flex flex-wrap gap-2 p-1 rounded-lg border w-fit transition-colors" :class="theme === 'dark' ? 'bg-industrial-800 border-industrial-600' : 'bg-gray-100 border-gray-200'">
      <button
        v-for="tab in tabs"
        :key="tab.value"
        class="px-4 py-2 rounded-md text-sm font-medium transition-all"
        :class="activeTab === tab.value
          ? 'bg-cyber-blue/20 text-cyber-blue border border-cyber-blue/40'
          : (theme === 'dark' ? 'text-gray-400 hover:text-white' : 'text-gray-500 hover:text-gray-900')"
        @click="activeTab = tab.value"
      >
        {{ tab.label }}
      </button>
    </div>

    <!-- 任务列表卡片 -->
    <div class="data-card rounded-xl p-0 neon-border overflow-hidden">
      <!-- 桌面端表格视图 -->
      <div class="hidden md:block overflow-x-auto">
        <table class="w-full">
          <thead>
            <tr class="border-b transition-colors" :class="theme === 'dark' ? 'border-industrial-600 bg-industrial-800/50' : 'border-gray-200 bg-gray-50'">
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('tasks.taskId') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('tasks.targetWaypoint') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('tasks.taskType') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('common.robot') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('common.status') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('common.priority') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('common.createdAt') }}</th>
              <th class="text-left py-3 px-4 text-xs uppercase tracking-wider font-medium" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('common.action') }}</th>
            </tr>
          </thead>
          <tbody>
            <tr
              v-for="task in filteredTasks"
              :key="task.id"
              class="border-b transition-colors"
              :class="theme === 'dark' ? 'border-industrial-700/50 hover:bg-industrial-700/30' : 'border-gray-100 hover:bg-gray-50'"
            >
              <td class="py-3 px-4">
                <span class="font-medium text-cyber-blue">{{ task.id }}</span>
              </td>
              <td class="py-3 px-4" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ task.waypoint_id }}</td>
              <td class="py-3 px-4">
                <span class="px-2 py-0.5 rounded text-xs font-medium" :class="taskTypeClass(task.task_type)">
                  {{ taskTypeName(task.task_type) }}
                </span>
              </td>
              <td class="py-3 px-4">
                <span v-if="task.robot_id" class="px-2.5 py-1 rounded-full text-xs font-medium bg-cyber-blue/20 text-cyber-blue">
                  {{ task.robot_id }}
                </span>
                <span v-else class="italic" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('tasks.unassigned') }}</span>
              </td>
              <td class="py-3 px-4">
                <span
                  class="inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium"
                  :class="taskStatusClass(task.status)"
                >
                  <span class="w-1.5 h-1.5 rounded-full" :class="taskStatusDotClass(task.status)" />
                  {{ statusText(task.status) }}
                </span>
              </td>
              <td class="py-3 px-4">
                <div class="flex gap-1">
                  <span
                    v-for="n in 3"
                    :key="n"
                    class="w-2 h-2 rounded-full transition-all"
                    :class="n <= task.priority ? 'bg-cyber-orange' : (theme === 'dark' ? 'bg-industrial-700' : 'bg-gray-200')"
                  />
                </div>
              </td>
              <td class="py-3 px-4 text-sm font-mono" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ task.created_at }}</td>
              <td class="py-3 px-4">
                <div class="flex gap-2">
                  <button
                    v-if="task.status === 'pending'"
                    class="px-3 py-1.5 rounded-lg text-xs font-medium bg-cyber-blue/20 text-cyber-blue hover:bg-cyber-blue/30 transition-colors"
                    @click="assignTask(task)"
                  >
                    {{ t('tasks.assign') }}
                  </button>
                  <button
                    v-if="task.status === 'running'"
                    class="px-3 py-1.5 rounded-lg text-xs font-medium bg-cyber-orange/20 text-cyber-orange hover:bg-cyber-orange/30 transition-colors"
                    @click="pauseTask(task.id)"
                  >
                    {{ t('tasks.pause') }}
                  </button>
                  <button
                    v-if="['pending', 'assigned', 'running', 'in_progress', 'waiting_fleet', 'executing'].includes(task.status)"
                    class="px-3 py-1.5 rounded-lg text-xs font-medium bg-cyber-red/20 text-cyber-red hover:bg-cyber-red/30 transition-colors"
                    @click="cancelTask(task.id)"
                  >
                    {{ t('tasks.cancel') }}
                  </button>
                  <button
                    v-if="task.status === 'completed' || task.status === 'failed'"
                    class="px-3 py-1.5 rounded-lg text-xs font-medium transition-colors"
                    :class="theme === 'dark' ? 'bg-industrial-700 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 text-gray-600 hover:bg-gray-200'"
                    @click="viewDetails(task)"
                  >
                    {{ t('tasks.viewDetails') }}
                  </button>
                </div>
              </td>
            </tr>
            <!-- 空数据占位 -->
            <tr v-if="filteredTasks.length === 0">
              <td colspan="8" class="py-12 text-center">
                <Inbox class="w-12 h-12 mx-auto mb-3" :class="theme === 'dark' ? 'text-gray-600' : 'text-gray-300'" />
                <p :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.noTasks') }}</p>
              </td>
            </tr>
          </tbody>
        </table>
      </div>

      <!-- 移动端卡片视图 -->
      <div class="md:hidden divide-y" :class="theme === 'dark' ? 'divide-industrial-700/50' : 'divide-gray-100'">
        <div
          v-for="task in filteredTasks"
          :key="task.id"
          class="p-4 space-y-3"
        >
          <div class="flex items-center justify-between">
            <span class="font-medium text-cyber-blue text-sm">{{ task.id }}</span>
            <span
              class="inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-xs font-medium"
              :class="taskStatusClass(task.status)"
            >
              <span class="w-1.5 h-1.5 rounded-full" :class="taskStatusDotClass(task.status)" />
              {{ statusText(task.status) }}
            </span>
          </div>
          <div class="flex items-center justify-between text-sm">
            <span :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('common.target') }}: {{ task.waypoint_id }}</span>
            <span class="px-2 py-0.5 rounded text-xs font-medium" :class="taskTypeClass(task.task_type)">
              {{ taskTypeName(task.task_type) }}
            </span>
            <span v-if="task.robot_id" class="px-2 py-0.5 rounded-full text-xs font-medium bg-cyber-blue/20 text-cyber-blue">
              {{ task.robot_id }}
            </span>
            <span v-else class="text-xs italic" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('tasks.unassigned') }}</span>
          </div>
          <div class="flex items-center justify-between text-xs" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
            <div class="flex items-center gap-2">
              <span>{{ t('common.priority') }}:</span>
              <div class="flex gap-1">
                <span
                  v-for="n in 3"
                  :key="n"
                  class="w-1.5 h-1.5 rounded-full"
                  :class="n <= task.priority ? 'bg-cyber-orange' : (theme === 'dark' ? 'bg-industrial-700' : 'bg-gray-200')"
                />
              </div>
            </div>
            <span class="font-mono">{{ task.created_at }}</span>
          </div>
          <!-- 移动端操作按钮 -->
          <div class="flex gap-2 pt-1">
            <button
              v-if="task.status === 'pending'"
              class="flex-1 px-3 py-2 rounded-lg text-xs font-medium bg-cyber-blue/20 text-cyber-blue hover:bg-cyber-blue/30 transition-colors"
              @click="assignTask(task)"
            >
              {{ t('tasks.assign') }}
            </button>
            <button
              v-if="task.status === 'running'"
              class="flex-1 px-3 py-2 rounded-lg text-xs font-medium bg-cyber-orange/20 text-cyber-orange hover:bg-cyber-orange/30 transition-colors"
              @click="pauseTask(task.id)"
            >
              {{ t('tasks.pause') }}
            </button>
            <button
              v-if="['pending', 'assigned', 'running', 'in_progress', 'waiting_fleet', 'executing'].includes(task.status)"
              class="flex-1 px-3 py-2 rounded-lg text-xs font-medium bg-cyber-red/20 text-cyber-red hover:bg-cyber-red/30 transition-colors"
              @click="cancelTask(task.id)"
            >
              {{ t('tasks.cancel') }}
            </button>
            <button
              v-if="task.status === 'completed' || task.status === 'failed'"
              class="flex-1 px-3 py-2 rounded-lg text-xs font-medium transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 text-gray-600 hover:bg-gray-200'"
              @click="viewDetails(task)"
            >
              {{ t('tasks.viewDetails') }}
            </button>
          </div>
        </div>
        <!-- 移动端空数据占位 -->
        <div v-if="filteredTasks.length === 0" class="py-12 text-center">
          <Inbox class="w-12 h-12 mx-auto mb-3" :class="theme === 'dark' ? 'text-gray-600' : 'text-gray-300'" />
          <p :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.noTasks') }}</p>
        </div>
      </div>
    </div>

    <!-- 新建任务弹窗 -->
    <div v-if="showNewTaskModal" class="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center p-4" @click.self="showNewTaskModal = false">
      <div class="data-card rounded-xl p-6 neon-border w-full max-w-md animate-scale-in">
        <div class="flex items-center justify-between mb-6">
          <h2 class="text-xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('tasks.newTask') }}</h2>
          <button class="p-2 rounded-lg transition-colors" :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'" @click="showNewTaskModal = false">
            <X class="w-5 h-5" />
          </button>
        </div>

        <div class="space-y-4">
          <!-- 目标航点选择 -->
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.targetWaypoint') }}</label>
            <select
              v-model="newTask.waypoint_id"
              class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
            >
              <option value="">{{ t('tasks.selectWaypoint') }}</option>
              <option v-for="wp in waypoints" :key="wp.id" :value="wp.id">
                {{ wp.id }} - {{ wp.name }}
              </option>
            </select>
          </div>

          <!-- 优先级选择 -->
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('common.priority') }}</label>
            <div class="flex gap-2">
              <button
                v-for="p in 3"
                :key="p"
                class="flex-1 py-2.5 rounded-lg text-sm font-medium transition-all"
                :class="newTask.priority >= p
                  ? 'bg-cyber-orange/20 border border-cyber-orange/40 text-cyber-orange'
                  : (theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-gray-400' : 'bg-gray-100 border border-gray-200 text-gray-500')"
                @click="newTask.priority = p"
              >
                {{ p }}{{ t('common.level') }}
              </button>
            </div>
          </div>

          <!-- 任务类型选择 -->
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.taskType') }}</label>
            <div class="grid grid-cols-2 gap-2">
              <button
                v-for="tt in taskTypes"
                :key="tt.value"
                class="py-2.5 rounded-lg text-sm font-medium transition-all"
                :class="newTask.task_type === tt.value
                  ? 'bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue'
                  : (theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-gray-400' : 'bg-gray-100 border border-gray-200 text-gray-500')"
                @click="newTask.task_type = tt.value; if (tt.value === 1) newTask.site_code = 0"
              >
                {{ tt.label }}
              </button>
            </div>
          </div>

          <!-- 站点编码输入（巡航类型时隐藏） -->
          <div v-if="newTask.task_type !== 1">
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.siteCode') }}</label>
            <input
              v-model.number="newTask.site_code"
              type="number"
              min="0"
              :placeholder="t('tasks.siteCodePlaceholder')"
              class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
            />
          </div>

          <!-- 机器人选择 -->
          <div>
            <label class="block text-sm mb-2" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.selectRobot') }}</label>
            <select
              v-model="newTask.robot_id"
              class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
            >
              <option value="">{{ t('tasks.autoAssign') }}</option>
              <option v-for="robot in availableRobots" :key="robot.id" :value="robot.id">
                {{ robot.id }} ({{ t('fleet.battery') }}: {{ robot.battery }}%)
              </option>
            </select>
          </div>

          <!-- 操作按钮 -->
          <div class="flex gap-3 pt-4">
            <button
              class="flex-1 px-4 py-2.5 rounded-lg border transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border-gray-200 text-gray-600 hover:bg-gray-200'"
              @click="showNewTaskModal = false"
            >
              {{ t('common.cancel') }}
            </button>
            <button
              class="flex-1 px-4 py-2.5 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue hover:bg-cyber-blue/30 transition-colors font-medium"
              @click="createTask"
            >
              {{ t('common.create') }}
            </button>
          </div>
        </div>
      </div>
    </div>

    <!-- 分配任务弹窗 -->
    <div v-if="showAssignModal" class="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center p-4" @click.self="showAssignModal = false">
      <div class="data-card rounded-xl p-6 neon-border w-full max-w-md animate-scale-in">
        <div class="flex items-center justify-between mb-6">
          <h2 class="text-xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('tasks.assignTask') }}</h2>
          <button class="p-2 rounded-lg transition-colors" :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'" @click="showAssignModal = false">
            <X class="w-5 h-5" />
          </button>
        </div>

        <!-- 任务信息提示 -->
        <p class="mb-4" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
          {{ t('tasks.taskId') }} <span class="text-cyber-blue font-medium">{{ taskToAssign?.id }}</span> → {{ t('common.target') }}: {{ taskToAssign?.waypoint_id }}
        </p>

        <!-- 可选机器人列表 -->
        <div class="space-y-2 max-h-64 overflow-y-auto mb-4">
          <div
            v-for="robot in availableRobots"
            :key="robot.id"
            class="p-3 rounded-lg border cursor-pointer transition-all"
            :class="selectedRobot === robot.id
              ? 'border-cyber-blue bg-cyber-blue/10'
              : (theme === 'dark' ? 'border-industrial-600 bg-industrial-700/50 hover:border-industrial-500' : 'border-gray-200 bg-gray-50 hover:border-gray-300')"
            @click="selectedRobot = robot.id"
          >
            <div class="flex items-center justify-between">
              <span class="font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ robot.id }}</span>
              <span class="text-xs px-2 py-0.5 rounded-full" :class="robot.status === 'online' ? 'bg-cyber-green/20 text-cyber-green' : 'bg-gray-500/20 text-gray-400'">
                {{ statusText(robot.status) }}
              </span>
            </div>
            <div class="flex items-center gap-4 mt-2 text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              <span class="flex items-center gap-1">
                <Battery class="w-3 h-3" /> {{ robot.battery }}%
              </span>
              <span class="flex items-center gap-1">
                <MapPin class="w-3 h-3" /> {{ robot.position || t('common.unknown') }}
              </span>
            </div>
          </div>
        </div>

        <!-- 操作按钮 -->
        <div class="flex gap-3">
          <button
            class="flex-1 px-4 py-2.5 rounded-lg border transition-colors"
            :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border-gray-200 text-gray-600 hover:bg-gray-200'"
            @click="showAssignModal = false"
          >
            {{ t('common.cancel') }}
          </button>
          <button
            class="flex-1 px-4 py-2.5 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue hover:bg-cyber-blue/30 transition-colors font-medium"
            @click="confirmAssign"
          >
            {{ t('tasks.confirmAssign') }}
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, reactive, onMounted } from 'vue'
import { storeToRefs } from 'pinia'
import { useFleetStore } from '../stores/fleet'
import { useSettingsStore } from '../stores/settings'
import { Plus, Clock, Loader2, CheckCircle, XCircle, X, Inbox, Battery, MapPin } from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const { language, theme } = storeToRefs(settingsStore)
const { t } = settingsStore

/** ROS task_status 中与"已接手/在执行路径"相近的状态（不含纯 pending） */
const RUNNING_LIKE = ['running', 'assigned', 'in_progress', 'waiting_fleet', 'executing']

const activeTab = ref('all')
const showNewTaskModal = ref(false)
const showAssignModal = ref(false)
const taskToAssign = ref(null)
const selectedRobot = ref('')

// 任务状态切换标签配置
const tabs = computed(() => [
  { value: 'all', label: t('tasks.allTasks') },
  { value: 'pending', label: t('tasks.pending') },
  { value: 'running', label: t('tasks.running') },
  { value: 'completed', label: t('tasks.completed') }
])

// 新建任务表单数据
const newTask = reactive({
  waypoint_id: '',
  priority: 1,
  robot_id: '',
  task_type: 1,
  site_code: 0
})

// 任务类型选项
const taskTypes = computed(() => [
  { value: 1, label: t('tasks.taskTypeCRUISE') },
  { value: 2, label: t('tasks.taskTypeLOAD') },
  { value: 3, label: t('tasks.taskTypeUNLOAD') },
  { value: 4, label: t('tasks.taskTypeSITE_SPECIFIC') }
])

const waypoints = ref([])

/** 从后端加载可用航点列表 */
async function loadWaypoints() {
  try {
    const response = await fetch('/api/map/waypoints')
    if (response.ok) {
      const data = await response.json()
      waypoints.value = data.waypoints || []
    }
  } catch (e) {
    console.error('Failed to load waypoints:', e)
  }
}

// 任务统计信息
const taskStats = computed(() => {
  const tasks = fleetStore.tasks
  return {
    pending: tasks.filter(t => t.status === 'pending').length,
    running: tasks.filter(t => RUNNING_LIKE.includes(t.status)).length,
    completed: tasks.filter(t => t.status === 'completed').length,
    failed: tasks.filter(t => t.status === 'failed').length
  }
})

// 根据活跃标签过滤并排序任务列表
const filteredTasks = computed(() => {
  let tasks = fleetStore.tasks
  if (activeTab.value !== 'all') {
    if (activeTab.value === 'running') {
      tasks = tasks.filter(t => RUNNING_LIKE.includes(t.status))
    } else {
      tasks = tasks.filter(t => t.status === activeTab.value)
    }
  }
  // 时间字符串转时间戳辅助函数
  const toTs = (s) => {
    if (!s) return 0
    // ISO 优先
    const d1 = new Date(s)
    if (!Number.isNaN(d1.getTime())) return d1.getTime()
    // 兼容旧的 HH:MM:SS
    if (/^\d{2}:\d{2}:\d{2}$/.test(s)) {
      const today = new Date()
      const iso = `${today.getFullYear()}-${String(today.getMonth() + 1).padStart(2, '0')}-${String(today.getDate()).padStart(2, '0')}T${s}`
      const d2 = new Date(iso)
      return Number.isNaN(d2.getTime()) ? 0 : d2.getTime()
    }
    return 0
  }
  return tasks.sort((a, b) => (b.priority - a.priority) || (toTs(b.created_at) - toTs(a.created_at)))
})

// 可用机器人列表（在线且空闲，按电量降序）
const availableRobots = computed(() => {
  return Object.entries(fleetStore.robots)
    .filter(([_, r]) => r.online && (r.status === 'idle' || r.status === 'arrived' || r.status === 'unknown'))
    .map(([id, r]) => ({ id, ...r }))
    .sort((a, b) => b.battery - a.battery)
})

/** 获取状态文本 */
function statusText(status) {
  const map = {
    pending: t('tasks.pending'),
    assigned: t('tasks.assigned'),
    running: t('tasks.running'),
    in_progress: t('tasks.running'),
    waiting_fleet: t('tasks.waiting_fleet'),
    executing: t('tasks.executing'),
    completed: t('tasks.completed'),
    failed: t('tasks.failed'),
    cancelled: t('tasks.taskCancelled'),
    idle: t('common.idle'),
    arrived: t('common.arrived'),
    moving: t('common.moving'),
    unknown: t('common.unknown')
  }
  return map[status] || status
}

/** 获取任务状态样式类 */
function taskStatusClass(status) {
  const classes = {
    pending: 'bg-gray-500/20 text-gray-400',
    assigned: 'bg-cyber-blue/20 text-cyber-blue',
    running: 'bg-cyber-green/20 text-cyber-green',
    in_progress: 'bg-cyber-green/20 text-cyber-green',
    waiting_fleet: 'bg-amber-500/20 text-amber-400',
    executing: 'bg-cyan-500/20 text-cyan-400',
    completed: 'bg-cyber-purple/20 text-cyber-purple',
    failed: 'bg-cyber-red/20 text-cyber-red',
    cancelled: 'bg-industrial-700/40 text-gray-400'
  }
  return classes[status] || 'bg-gray-500/20 text-gray-400'
}

/** 获取任务状态圆点样式类 */
function taskStatusDotClass(status) {
  const classes = {
    pending: 'bg-gray-500',
    assigned: 'bg-cyber-blue',
    running: 'bg-cyber-green',
    in_progress: 'bg-cyber-green',
    waiting_fleet: 'bg-amber-500',
    executing: 'bg-cyan-500',
    completed: 'bg-cyber-purple',
    failed: 'bg-cyber-red',
    cancelled: 'bg-gray-500'
  }
  return classes[status] || 'bg-gray-500'
}

/** 获取任务类型中文名 */
function taskTypeName(taskType) {
  const map = {
    1: t('tasks.taskTypeCRUISE'),
    2: t('tasks.taskTypeLOAD'),
    3: t('tasks.taskTypeUNLOAD'),
    4: t('tasks.taskTypeSITE_SPECIFIC')
  }
  return map[taskType] || String(taskType)
}

/** 获取任务类型样式类 */
function taskTypeClass(taskType) {
  const map = {
    1: 'bg-gray-500/20 text-gray-400',
    2: 'bg-cyber-blue/20 text-cyber-blue',
    3: 'bg-cyber-orange/20 text-cyber-orange',
    4: 'bg-cyber-purple/20 text-cyber-purple'
  }
  return map[taskType] || 'bg-gray-500/20 text-gray-400'
}

/** 创建新任务 */
async function createTask() {
  if (!newTask.waypoint_id) {
    alert(t('tasks.selectWaypoint'))
    return
  }
  try {
    const response = await fetch('/api/tasks', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        waypoint_id: newTask.waypoint_id,
        priority: newTask.priority,
        robot_id: newTask.robot_id || null,
        task_type: newTask.task_type,
        site_code: newTask.site_code
      })
    })
    if (response.ok) {
      const data = await response.json()
      fleetStore.addLog('success', `${t('common.create')} ${data.task_id}`)
      newTask.waypoint_id = ''
      newTask.priority = 1
      newTask.robot_id = ''
      newTask.task_type = 1
      newTask.site_code = 0
      showNewTaskModal.value = false
    } else {
      const error = await response.json()
      alert(`${t('tasks.createTaskFailed')}: ${error.detail || t('common.unknown')}`)
    }
  } catch (e) {
    console.error('Failed to create task:', e)
    alert(t('tasks.createTaskFailed'))
  }
}

/** 打开分配任务弹窗 */
function assignTask(task) {
  taskToAssign.value = task
  selectedRobot.value = ''
  showAssignModal.value = true
}

/** 确认分配任务给指定机器人 */
async function confirmAssign() {
  if (!selectedRobot.value) {
    alert(t('tasks.selectRobotFirst'))
    return
  }
  try {
    const response = await fetch(`/api/tasks/${taskToAssign.value.id}`, {
      method: 'PATCH',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ robot_id: selectedRobot.value })
    })
    if (response.ok) {
      fleetStore.addLog('info', `${t('tasks.taskId')} ${taskToAssign.value.id} \u2192 ${selectedRobot.value}`)
      showAssignModal.value = false
    } else {
      const error = await response.json()
      alert(`${t('tasks.assignFailed')}: ${error.detail || t('common.unknown')}`)
    }
  } catch (e) {
    console.error('Failed to assign:', e)
    alert(t('tasks.assignFailed'))
  }
}

/** 暂停任务（本地直接修改状态，无后端调用） */
function pauseTask(taskId) {
  const task = fleetStore.tasks.find(t => t.id === taskId)
  if (task) {
    task.status = 'pending'
    fleetStore.addLog('warning', `${t('tasks.taskId')} ${taskId} ${t('tasks.taskPaused')}`)
  }
}

/** 取消任务 */
async function cancelTask(taskId) {
  if (confirm(t('tasks.confirmCancel'))) {
    try {
      const response = await fetch(`/api/tasks/${taskId}`, { method: 'DELETE' })
      if (!response.ok) {
        const error = await response.json()
        alert(`${t('tasks.cancel')}${t('common.error')}: ${error.detail || t('common.unknown')}`)
        return
      }
      const task = fleetStore.tasks.find(t => t.id === taskId)
      if (task) {
        task.status = 'cancelled'
      }
      fleetStore.addLog('info', `${t('tasks.taskId')} ${taskId} ${t('tasks.taskCancelled')}`)
    } catch (e) {
      console.error('Failed to cancel task:', e)
      alert(`${t('tasks.cancel')}${t('common.error')}`)
    }
  }
}

/** 查看任务详情（简易弹窗） */
function viewDetails(task) {
  alert(`${t('tasks.viewDetails')}:\nID: ${task.id}\n${t('common.target')}: ${task.waypoint_id}\n${t('common.status')}: ${statusText(task.status)}`)
}

onMounted(() => {
  loadWaypoints()
  // 检查是否有从其他页面预填的机器人 ID
  const prefill = localStorage.getItem('prefill_robot_id')
  if (prefill) {
    newTask.robot_id = prefill
    localStorage.removeItem('prefill_robot_id')
  }
})
</script>
