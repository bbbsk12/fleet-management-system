<template>
  <div class="space-y-4 sm:space-y-6">
    <!-- 页面头部：标题与创建任务按钮 + 强调线 -->
    <div class="border-b-2 border-cyber-blue/30 pb-4">
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

    <!-- 任务状态切换标签（现代药丸样式） -->
    <div class="flex gap-1 p-1.5 rounded-xl w-fit transition-colors" :class="theme === 'dark' ? 'bg-industrial-800' : 'bg-gray-100'">
      <button
        v-for="tab in tabs"
        :key="tab.value"
        class="px-5 py-2 rounded-lg text-sm font-medium transition-all"
        :class="activeTab === tab.value
          ? (theme === 'dark' ? 'bg-industrial-700 text-cyber-blue shadow-sm' : 'bg-white text-cyber-blue shadow-sm')
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
            <tr class="sticky top-0 z-10 border-b transition-colors" :class="theme === 'dark' ? 'border-industrial-600 bg-industrial-800/95 backdrop-blur-sm' : 'border-gray-200 bg-gray-50/95 backdrop-blur-sm'">
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
              v-for="(task, index) in filteredTasks"
              :key="task.id"
              class="border-b transition-colors"
              :class="[
                theme === 'dark' ? 'border-industrial-700/50' : 'border-gray-100',
                index % 2 === 0
                  ? (theme === 'dark' ? 'bg-industrial-900/20' : 'bg-white/50')
                  : (theme === 'dark' ? 'bg-industrial-800/10' : 'bg-gray-50/30'),
                'hover:bg-cyber-blue/5 dark:hover:bg-cyber-blue/10'
              ]"
            >
              <td class="py-3 px-4">
                <span class="font-medium text-cyber-blue text-sm">{{ task.id }}</span>
              </td>
              <td class="py-3 px-4">
                <span class="font-mono text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ task.waypoint_id }}</span>
              </td>
              <td class="py-3 px-4">
                <span class="px-2.5 py-1 rounded-md text-xs font-medium" :class="taskTypeClass(task.task_type)">
                  {{ taskTypeName(task.task_type) }}
                </span>
              </td>
              <td class="py-3 px-4">
                <span v-if="task.robot_id" class="inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium bg-cyber-blue/20 text-cyber-blue">
                  <Bot class="w-3 h-3" />
                  {{ task.robot_id }}
                </span>
                <span v-else class="italic text-sm" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('tasks.unassigned') }}</span>
              </td>
              <td class="py-3 px-4">
                <span
                  class="inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium whitespace-nowrap"
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
              <td colspan="8" class="py-16 text-center">
                <Inbox class="w-12 h-12 mx-auto mb-3" :class="theme === 'dark' ? 'text-gray-600' : 'text-gray-300'" />
                <p class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.noTasks') }}</p>
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
          class="p-4 space-y-3 transition-colors hover:bg-cyber-blue/5 dark:hover:bg-cyber-blue/10"
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
            <span v-if="task.robot_id" class="inline-flex items-center gap-1 px-2 py-0.5 rounded-full text-xs font-medium bg-cyber-blue/20 text-cyber-blue">
              <Bot class="w-3 h-3" />
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
        <div v-if="filteredTasks.length === 0" class="py-16 text-center">
          <Inbox class="w-12 h-12 mx-auto mb-3" :class="theme === 'dark' ? 'text-gray-600' : 'text-gray-300'" />
          <p class="text-sm" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('tasks.noTasks') }}</p>
        </div>
      </div>
    </div>

    <!-- 新建任务弹窗 -->
    <div v-if="showNewTaskModal" class="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center p-4" @click.self="showNewTaskModal = false">
      <div class="data-card rounded-xl p-6 neon-border w-full max-w-lg animate-scale-in">
        <div class="flex items-center justify-between mb-6">
          <h2 class="text-xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('tasks.newTask') }}</h2>
          <button class="p-2 rounded-lg transition-colors" :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'" @click="showNewTaskModal = false">
            <X class="w-5 h-5" />
          </button>
        </div>

        <div class="space-y-5">
          <!-- 目标航点选择 -->
          <div>
            <h4 class="text-xs uppercase tracking-wider font-semibold mb-3 flex items-center gap-2" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
              <MapPin class="w-3.5 h-3.5" />
              {{ t('tasks.targetWaypoint') }}
            </h4>
            <select
              v-model="newTask.waypoint_id"
              class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
            >
              <option value="">{{ t('tasks.selectWaypoint') }}</option>
              <option v-for="wp in waypoints" :key="wp.id" :value="wp.id">
                {{ wp.id }} - {{ wp.name || wp.id }}
              </option>
            </select>
            <!-- 目标航点预览 -->
            <div v-if="newTask.waypoint_id" class="mt-2 p-2.5 rounded-lg border text-xs" :class="theme === 'dark' ? 'bg-cyber-purple/10 border-cyber-purple/30 text-cyber-purple' : 'bg-purple-50 border-purple-200 text-purple-700'">
              <div class="flex items-center gap-2">
                <MapPin class="w-3.5 h-3.5 shrink-0" />
                <span class="font-medium">{{ waypoints.find(w => w.id === newTask.waypoint_id)?.name || newTask.waypoint_id }}</span>
              </div>
              <p class="mt-1 font-mono opacity-75">
                ({{ waypoints.find(w => w.id === newTask.waypoint_id)?.x?.toFixed(1) || '?' }}, {{ waypoints.find(w => w.id === newTask.waypoint_id)?.y?.toFixed(1) || '?' }})
              </p>
            </div>
          </div>

          <!-- 优先级选择 -->
          <div>
            <h4 class="text-xs uppercase tracking-wider font-semibold mb-3 flex items-center gap-2" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
              <svg class="w-3.5 h-3.5" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/></svg>
              {{ t('common.priority') }}
            </h4>
            <div class="flex gap-3">
              <button
                v-for="p in 3"
                :key="p"
                class="flex-1 py-3 rounded-xl text-sm font-medium transition-all relative overflow-hidden"
                :class="newTask.priority >= p
                  ? 'bg-cyber-orange/20 border-2 border-cyber-orange/50 text-cyber-orange'
                  : (theme === 'dark' ? 'bg-industrial-700 border-2 border-transparent text-gray-400 hover:border-industrial-500' : 'bg-gray-100 border-2 border-transparent text-gray-500 hover:border-gray-300')"
                @click="newTask.priority = p"
              >
                <div class="flex items-center justify-center gap-1.5">
                  <svg v-for="n in p" :key="n" class="w-3.5 h-3.5 fill-current" viewBox="0 0 24 24" stroke="none"><polygon points="12 2 15.09 8.26 22 9.27 17 14.14 18.18 21.02 12 17.77 5.82 21.02 7 14.14 2 9.27 8.91 8.26 12 2"/></svg>
                </div>
                <span class="block mt-1">{{ p }}{{ t('common.level') }}</span>
              </button>
            </div>
          </div>

          <!-- 任务类型选择 -->
          <div>
            <h4 class="text-xs uppercase tracking-wider font-semibold mb-3 flex items-center gap-2" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
              <svg class="w-3.5 h-3.5" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><rect x="3" y="3" width="7" height="7"/><rect x="14" y="3" width="7" height="7"/><rect x="14" y="14" width="7" height="7"/><rect x="3" y="14" width="7" height="7"/></svg>
              {{ t('tasks.taskType') }}
            </h4>
            <div class="grid grid-cols-2 gap-2">
              <button
                v-for="tt in taskTypes"
                :key="tt.value"
                class="py-3 rounded-lg text-sm font-medium transition-all"
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
            <h4 class="text-xs uppercase tracking-wider font-semibold mb-3 flex items-center gap-2" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
              <svg class="w-3.5 h-3.5" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round"><polyline points="4 7 4 4 20 4 20 7"/><line x1="9" y1="20" x2="15" y2="20"/><line x1="12" y1="4" x2="12" y2="20"/></svg>
              {{ t('tasks.siteCode') }}
            </h4>
            <input
              v-model.number="newTask.site_code"
              type="number"
              min="0"
              :placeholder="t('tasks.siteCodePlaceholder')"
              class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
            />
          </div>

          <!-- 机器人选择 -->
          <div>
            <h4 class="text-xs uppercase tracking-wider font-semibold mb-3 flex items-center gap-2" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
              <Bot class="w-3.5 h-3.5" />
              {{ t('tasks.selectRobot') }}
            </h4>
            <select
              v-model="newTask.robot_id"
              class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none transition-colors"
              :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
            >
              <option value="">{{ t('tasks.autoAssign') }}</option>
              <option v-for="robot in availableRobots" :key="robot.id" :value="robot.id">
                {{ robot.id }} ({{ t('fleet.battery') }}: {{ robot.battery }}%)
              </option>
            </select>
          </div>

          <!-- 操作按钮 -->
          <div class="flex gap-3 pt-4 border-t" :class="theme === 'dark' ? 'border-industrial-700' : 'border-gray-200'">
            <button
              class="flex-1 px-4 py-2.5 rounded-lg border transition-colors font-medium"
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
      <div class="data-card rounded-xl p-6 neon-border w-full max-w-lg animate-scale-in">
        <div class="flex items-center justify-between mb-6">
          <h2 class="text-xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('tasks.assignTask') }}</h2>
          <button class="p-2 rounded-lg transition-colors" :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'" @click="showAssignModal = false">
            <X class="w-5 h-5" />
          </button>
        </div>

        <!-- 任务信息提示 -->
        <div class="mb-5 p-3 rounded-lg border" :class="theme === 'dark' ? 'bg-cyber-blue/10 border-cyber-blue/30' : 'bg-blue-50 border-blue-200'">
          <p class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">
            <span class="text-cyber-blue font-medium">{{ taskToAssign?.id }}</span>
            <span class="mx-2 text-gray-400">→</span>
            <span>{{ t('common.target') }}: <strong>{{ taskToAssign?.waypoint_id }}</strong></span>
          </p>
        </div>

        <!-- 可选机器人列表 -->
        <label class="block text-sm mb-3 font-medium" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('tasks.selectRobot') }}</label>
        <div class="space-y-3 max-h-72 overflow-y-auto mb-5">
          <div
            v-for="robot in availableRobots"
            :key="robot.id"
            class="p-4 rounded-xl border-2 cursor-pointer transition-all"
            :class="selectedRobot === robot.id
              ? 'border-cyber-blue bg-cyber-blue/10 ring-2 ring-cyber-blue/20'
              : (theme === 'dark' ? 'border-industrial-600 bg-industrial-700/50 hover:border-industrial-500' : 'border-gray-200 bg-gray-50 hover:border-gray-300')"
            @click="selectedRobot = robot.id"
          >
            <div class="flex items-center justify-between">
              <div class="flex items-center gap-3">
                <!-- 机器人头像图标 -->
                <div class="flex items-center justify-center w-11 h-11 rounded-full" :class="selectedRobot === robot.id ? 'bg-cyber-blue/20' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-200')">
                  <Bot class="w-5 h-5" :class="selectedRobot === robot.id ? 'text-cyber-blue' : (theme === 'dark' ? 'text-gray-400' : 'text-gray-500')" />
                </div>
                <div>
                  <div class="flex items-center gap-2">
                    <span class="font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ robot.id }}</span>
                    <span class="inline-flex items-center gap-1 px-2 py-0.5 rounded-full text-xs font-medium" :class="robot.status === 'online' || robot.status === 'idle' || robot.status === 'arrived' ? 'bg-cyber-green/20 text-cyber-green' : 'bg-gray-500/20 text-gray-400'">
                      <span class="w-1.5 h-1.5 rounded-full" :class="robot.status === 'online' || robot.status === 'idle' || robot.status === 'arrived' ? 'bg-cyber-green' : 'bg-gray-500'"></span>
                      {{ statusText(robot.status) }}
                    </span>
                  </div>
                  <div class="flex items-center gap-3 mt-1.5 text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
                    <span class="flex items-center gap-1">
                      <Battery class="w-3 h-3" />
                      {{ robot.battery }}%
                    </span>
                    <span class="flex items-center gap-1">
                      <MapPin class="w-3 h-3" />
                      {{ robot.position ? `(${robot.position.x?.toFixed(0)}, ${robot.position.y?.toFixed(0)})` : t('common.unknown') }}
                    </span>
                  </div>
                </div>
              </div>
              <!-- 选中勾选标记 -->
              <div v-if="selectedRobot === robot.id" class="w-6 h-6 rounded-full bg-cyber-blue flex items-center justify-center shrink-0">
                <svg class="w-3.5 h-3.5 text-white" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="3" stroke-linecap="round" stroke-linejoin="round">
                  <polyline points="20 6 9 17 4 12"></polyline>
                </svg>
              </div>
            </div>
            <!-- 电量条 -->
            <div class="mt-3 h-1.5 rounded-full overflow-hidden" :class="theme === 'dark' ? 'bg-industrial-700' : 'bg-gray-200'">
              <div
                class="h-full rounded-full transition-all"
                :class="robot.battery > 60 ? 'bg-cyber-green' : robot.battery > 20 ? 'bg-cyber-orange' : 'bg-cyber-red'"
                :style="{ width: robot.battery + '%' }"
              ></div>
            </div>
          </div>
        </div>

        <!-- 操作按钮 -->
        <div class="flex gap-3 pt-2">
          <button
            class="flex-1 px-5 py-3 rounded-xl border transition-colors font-medium"
            :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border-gray-200 text-gray-600 hover:bg-gray-200'"
            @click="showAssignModal = false"
          >
            {{ t('common.cancel') }}
          </button>
          <button
            class="flex-1 px-5 py-3 rounded-xl bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue hover:bg-cyber-blue/30 transition-colors font-medium"
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
import { Plus, Clock, Loader2, CheckCircle, XCircle, X, Inbox, Battery, MapPin, Bot } from 'lucide-vue-next'

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
