<template>
  <div class="space-y-4 sm:space-y-6">
    <!-- 页面头部：标题与操作按钮 -->
    <div class="flex items-center justify-between flex-wrap gap-2 sm:gap-4">
      <div>
        <h1 class="text-xl sm:text-2xl font-bold tracking-wide" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('map.title') }}</h1>
        <p class="text-xs sm:text-sm mt-0.5" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-500'">{{ t('map.subtitle') }}</p>
      </div>
    </div>

    <!-- 地图 Canvas 容器 -->
    <div class="data-card rounded-xl p-0 neon-border overflow-hidden">
      <div class="relative h-[300px] sm:h-[400px] lg:h-[550px] cursor-grab active:cursor-grabbing" :class="theme === 'dark' ? 'bg-industrial-900' : 'bg-gray-100'" ref="mapWrapper">
        <!-- 地图绘制 Canvas -->
        <canvas
          ref="mapCanvas"
          class="absolute inset-0 w-full h-full"
          @click="handleMapClick"
          @wheel="handleWheel"
          @mousedown="startDrag"
          @mousemove="doDrag"
          @mouseup="endDrag"
          @touchstart.passive="onTouchStart"
          @touchmove.prevent="onTouchMove"
          @touchend="onTouchEnd"
        ></canvas>

        <!-- 浮动工具栏：加载地图与刷新位置 -->
        <div class="absolute top-4 left-4 z-30 flex gap-2 bg-black/40 backdrop-blur-md rounded-xl p-2">
          <button
            class="px-3 py-2 rounded-lg text-sm transition-colors flex items-center gap-2 text-white/90 hover:bg-white/10"
            @click="loadMapFile"
          >
            <FolderOpen class="w-4 h-4" />
            {{ t('map.loadMap') }}
          </button>
          <button
            class="px-3 py-2 rounded-lg text-sm transition-colors flex items-center gap-2 text-cyber-blue hover:bg-white/10"
            @click="refreshPositions"
          >
            <RotateCcw class="w-4 h-4" />
            {{ t('map.refreshPositions') }}
          </button>
        </div>

        <!-- 航点标记：锚点仅为圆点 32x32；标签绝对定位在下方，避免长文案撑宽容器导致 translate 中心偏移 -->
        <div
          v-for="wp in waypoints"
          :key="wp.id"
          class="absolute z-10 overflow-visible"
          :style="getWaypointAnchorStyle(wp)"
        >
          <div class="relative h-8 w-8 cursor-pointer transition-transform hover:scale-125 origin-center" @click.stop="selectWaypoint(wp)">
            <div
              class="h-8 w-8 rounded-full flex items-center justify-center shadow-lg"
              :class="getWaypointBgClass(wp)"
            >
              <component :is="getWaypointIcon(wp)" class="w-4 h-4 text-white" />
            </div>
            <div
              class="absolute top-full left-1/2 z-20 mt-1 max-w-[min(12rem,40vw)] -translate-x-1/2 cursor-pointer truncate px-1.5 py-0.5 text-center text-xs font-medium rounded"
              :class="theme === 'dark' ? 'text-white bg-black/60' : 'text-gray-900 bg-white/80'"
              :title="wp.name || wp.id"
              @click.stop="selectWaypoint(wp)"
            >
              {{ wp.name || wp.id }}
            </div>
          </div>
        </div>

        <!-- 链式撤退状态指示器 -->
        <div v-if="chainActive" class="fixed top-20 right-4 z-40 data-card rounded-xl p-4 neon-border border-cyber-orange/50 animate-pulse max-w-xs">
          <div class="flex items-center gap-2 mb-2">
            <RefreshCw class="w-5 h-5 text-cyber-orange animate-spin" />
            <span class="font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('map.chainActive') }}</span>
          </div>
          <p class="text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
            {{ t('map.chainDesc') }}
          </p>
        </div>

        <!-- 机器人标记 -->
        <div
          v-for="robot in robotMarkers"
          :key="robot.id"
          class="absolute z-20 overflow-visible"
          :style="getRobotAnchorStyle(robot)"
        >
          <div
            class="relative h-10 w-10 cursor-pointer transition-transform hover:scale-110 origin-center"
            @click.stop="selectRobot(robot)"
          >
            <!-- 机器人朝向指示箭头 -->
            <div class="absolute inset-0 pointer-events-none" :style="getRobotHeadingStyle(robot)">
              <div class="absolute left-1/2 top-1/2 -translate-x-1/2 -translate-y-1/2 scale-[2.6]">
                <div class="relative h-5 w-5" :class="getRobotArrowGlowClass(robot)">
                  <div class="absolute left-1/2 top-0 -translate-x-1/2 h-0 w-0 border-x-[7px] border-x-transparent border-b-[14px]" :class="getRobotArrowClass(robot)"></div>
                </div>
              </div>
            </div>
            <!-- offline 标记 -->
            <div
              v-if="!robot.online || robot.status === 'offline'"
              class="absolute -top-1 -right-1 px-1 py-0.5 text-[8px] font-bold rounded bg-cyber-red text-white leading-none"
            >
              OFF
            </div>
            <!-- 机器人信息标签 -->
            <div
              class="absolute top-full left-1/2 z-20 mt-1 flex max-w-[min(12rem,42vw)] -translate-x-1/2 cursor-pointer flex-col items-center gap-0.5"
              @click.stop="selectRobot(robot)"
            >
              <div
                class="truncate px-2 py-0.5 text-center text-xs font-bold rounded w-full"
                :class="theme === 'dark' ? 'text-white bg-black/70' : 'text-gray-900 bg-white/80'"
                :title="robot.id"
              >
                {{ robot.id }}
              </div>
              <div
                v-if="robot.location_type === 'waypoint' && robot.current_waypoint"
                class="truncate px-1.5 py-0.5 text-center text-[10px] rounded w-full"
                :class="theme === 'dark' ? 'text-cyber-green bg-cyber-green/20' : 'text-green-600 bg-green-100'"
                :title="'@' + robot.current_waypoint"
              >
                @{{ robot.current_waypoint }}
              </div>
              <div
                v-else-if="robot.location_type === 'segment' && robot.current_segment"
                class="truncate px-1.5 py-0.5 text-center text-[10px] rounded w-full"
                :class="theme === 'dark' ? 'text-cyber-blue bg-cyber-blue/20' : 'text-blue-600 bg-blue-100'"
              >
                → {{ robot.current_segment }} · {{ t('fleet.segmentMoving') }}
              </div>
              <div
                class="text-[10px] text-center whitespace-nowrap"
                :class="getBatteryTextClass(robot.battery)"
              >
                {{ robot.battery }}%
              </div>
            </div>
          </div>
        </div>

        <!-- 机器人信息浮窗（选中时显示在机器人标记附近） -->
        <div
          v-if="selectedRobot"
          class="absolute z-30 w-72 sm:w-80 bg-black/60 backdrop-blur-xl rounded-xl p-4 border border-white/10 shadow-2xl"
          :style="{
            left: Math.min(Math.max(((selectedRobot.position?.x || 0) * scale + offset.x + 30), 10), (mapWrapper?.clientWidth || 400) - 320) + 'px',
            top: Math.max(((selectedRobot.position?.y || 0) * scale + offset.y - 140), 10) + 'px'
          }"
        >
          <div class="flex items-center justify-between mb-3">
            <h3 class="text-base font-bold flex items-center gap-2 text-white">
              <Bot class="w-4 h-4 text-cyber-blue" />
              {{ selectedRobot.id }} {{ t('map.robotDetail') }}
            </h3>
            <button class="p-1.5 rounded-lg transition-colors hover:bg-white/10 text-gray-400" @click="selectedRobot = null">
              <X class="w-4 h-4" />
            </button>
          </div>
          <div class="grid grid-cols-2 gap-3 text-sm">
            <div>
              <p class="text-xs mb-1 text-gray-400">{{ t('common.status') }}</p>
              <span class="inline-flex items-center gap-1.5 px-2 py-0.5 rounded-full text-xs font-medium" :class="robotStatusClass(selectedRobot.status)">
                <span class="w-1.5 h-1.5 rounded-full" :class="robotStatusDotClass(selectedRobot.status)" />
                {{ statusText(selectedRobot.status) }}
              </span>
            </div>
            <div>
              <p class="text-xs mb-1 text-gray-400">{{ t('fleet.battery') }}</p>
              <div class="flex items-center gap-2">
                <div class="flex-1 h-1.5 rounded-full overflow-hidden bg-white/10">
                  <div
                    class="h-full rounded-full transition-all"
                    :class="batteryBarClass(selectedRobot.battery)"
                    :style="{ width: selectedRobot.battery + '%' }"
                  ></div>
                </div>
                <span class="text-xs text-gray-300">{{ selectedRobot.battery }}%</span>
              </div>
            </div>
            <div>
              <p class="text-xs mb-1 text-gray-400">{{ t('fleet.position') }}</p>
              <p class="text-xs font-mono text-gray-300">
                ({{ selectedRobot.position?.x?.toFixed(1) }}, {{ selectedRobot.position?.y?.toFixed(1) }})
              </p>
            </div>
            <div>
              <p class="text-xs mb-1 text-gray-400">{{ t('fleet.currentTask') }}</p>
              <p class="text-xs text-gray-300 truncate">{{ selectedRobot.current_task || t('common.none') }}</p>
            </div>
          </div>
          <!-- 机器人操作按钮 -->
          <div class="flex flex-wrap gap-2 mt-3 pt-3 border-t border-white/10">
            <button class="flex-1 px-3 py-1.5 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue text-xs hover:bg-cyber-blue/30 transition-colors" @click="sendToWaypoint">
              {{ t('map.sendToWaypoint') }}
            </button>
            <button class="flex-1 px-3 py-1.5 rounded-lg bg-cyber-green/20 border border-cyber-green/40 text-cyber-green text-xs hover:bg-cyber-green/30 transition-colors" @click="returnToCharge">
              {{ t('map.returnToCharge') }}
            </button>
            <button class="px-3 py-1.5 rounded-lg bg-cyber-red/20 border border-cyber-red/40 text-cyber-red text-xs hover:bg-cyber-red/30 transition-colors" @click="stopRobot">
              {{ t('map.emergencyStop') }}
            </button>
          </div>
        </div>

        <!-- 图例 + 缩放比例尺（左下角） -->
        <div class="absolute bottom-4 left-4 z-30 flex flex-col gap-1.5">
          <div class="flex flex-wrap gap-x-3 gap-y-1 bg-black/40 backdrop-blur-md rounded-lg px-3 py-2 text-xs text-white/90">
            <span class="flex items-center gap-1.5"><span class="w-2.5 h-2.5 rounded-full bg-cyber-blue"></span>{{ t('map.robot') }}</span>
            <span class="flex items-center gap-1.5"><span class="w-2.5 h-2.5 rounded-full bg-gray-500 opacity-60"></span>{{ t('map.offlineRobot', '离线底盘') }}</span>
            <span class="flex items-center gap-1.5"><span class="w-2.5 h-2.5 rounded-full bg-cyber-purple"></span>{{ t('map.waypoint') }}</span>
            <span class="flex items-center gap-1.5"><span class="w-2.5 h-2.5 rounded-full bg-cyber-green"></span>{{ t('map.chargingStation') }}</span>
            <span class="flex items-center gap-1.5"><span class="w-2.5 h-2.5 rounded-full bg-cyber-orange"></span>{{ t('map.parkingSpot') }}</span>
          </div>
          <div class="bg-black/40 backdrop-blur-md rounded-lg px-3 py-1 text-xs text-white/60 font-mono w-fit">
            {{ Math.round(scale * 100) }}%
          </div>
        </div>

        <!-- 缩放控制按钮（右下角） -->
        <div class="absolute bottom-4 right-4 z-30 flex flex-col gap-1">
          <button
            class="w-9 h-9 rounded-lg bg-black/40 backdrop-blur-md flex items-center justify-center text-white hover:bg-white/20 transition-colors"
            @click="zoomIn"
          >
            <Plus class="w-4 h-4" />
          </button>
          <button
            class="w-9 h-9 rounded-lg bg-black/40 backdrop-blur-md flex items-center justify-center text-white hover:bg-white/20 transition-colors"
            @click="zoomOut"
          >
            <Minus class="w-4 h-4" />
          </button>
        </div>
      </div>
    </div>

    <!-- 底部信息面板网格 -->
    <div class="grid grid-cols-1 lg:grid-cols-2 gap-4">
      <!-- 左侧面板：选中航点的详情 或 机器人列表 -->
      <div v-if="selectedWaypoint" class="data-card rounded-xl p-4 sm:p-6 neon-border">
        <div class="flex items-center justify-between mb-4">
          <h3 class="text-lg font-bold flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
            <MapPin class="w-5 h-5 text-cyber-purple" />
            {{ selectedWaypoint.name || selectedWaypoint.id }}
          </h3>
          <button class="p-2 rounded-lg transition-colors" :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'" @click="selectedWaypoint = null">
            <X class="w-4 h-4" />
          </button>
        </div>
        <div class="grid grid-cols-2 gap-4">
          <div>
            <p class="text-xs mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">ID</p>
            <p class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ selectedWaypoint.id }}</p>
          </div>
          <div>
            <p class="text-xs mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('fleet.position') }}</p>
            <p class="text-sm font-mono" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">
              ({{ selectedWaypoint.x?.toFixed(1) }}, {{ selectedWaypoint.y?.toFixed(1) }})
            </p>
          </div>
          <div>
            <p class="text-xs mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('map.type') }}</p>
            <span class="inline-flex items-center gap-1.5 px-2.5 py-1 rounded-full text-xs font-medium" :class="waypointTypeClass(selectedWaypoint)">
              <component :is="getWaypointIcon(selectedWaypoint)" class="w-3 h-3" />
              {{ getWaypointTypeName(selectedWaypoint) }}
            </span>
          </div>
          <div>
            <p class="text-xs mb-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">{{ t('map.connections') }}</p>
            <p class="text-sm" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-600'">{{ selectedWaypoint.connections?.length || 0 }}</p>
          </div>
        </div>
        <div class="mt-4 pt-4 border-t" :class="theme === 'dark' ? 'border-industrial-700' : 'border-gray-200'">
          <button class="px-4 py-2 rounded-lg bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue text-sm hover:bg-cyber-blue/30 transition-colors w-full" @click="createTaskHere">
            {{ t('map.dispatchRobot') }}
          </button>
        </div>
      </div>

      <!-- 机器人列表面板（可折叠） -->
      <details v-else class="data-card rounded-xl p-4 sm:p-6 neon-border group" open>
        <summary class="flex items-center justify-between cursor-pointer list-none [&::-webkit-details-marker]:hidden [&::marker]:hidden">
          <h3 class="text-lg font-bold flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
            <Bot class="w-5 h-5 text-cyber-blue" />
            {{ t('map.robotList') }}
          </h3>
          <div class="flex items-center gap-2">
            <span class="text-xs px-2 py-0.5 rounded-full bg-cyber-blue/20 text-cyber-blue">{{ robotList.length }}</span>
            <svg class="w-4 h-4 transition-transform group-open:rotate-180" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <polyline points="6 9 12 15 18 9"></polyline>
            </svg>
          </div>
        </summary>
        <div class="mt-4 space-y-2 max-h-48 overflow-y-auto">
          <div
            v-for="robot in robotList"
            :key="robot.id"
            class="p-3 rounded-lg border cursor-pointer transition-colors"
            :class="selectedRobot?.id === robot.id
              ? 'border-cyber-blue bg-cyber-blue/10'
              : (theme === 'dark' ? 'border-industrial-600 hover:border-cyber-blue bg-industrial-700/30' : 'border-gray-200 hover:border-cyber-blue bg-gray-50')"
            @click="selectRobot(robot)"
          >
            <div class="flex items-center justify-between">
              <span class="font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ robot.id }}</span>
              <span class="text-xs px-2 py-0.5 rounded-full" :class="robotStatusClass(robot.status)">
                {{ statusText(robot.status) }}
              </span>
            </div>
            <div class="flex items-center gap-4 mt-2 text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
              <span class="flex items-center gap-1">
                <Battery class="w-3 h-3" :class="getBatteryIconClass(robot.battery)" /> {{ robot.battery }}%
              </span>
              <span class="font-mono">
                ({{ robot.position?.x?.toFixed(0) }}, {{ robot.position?.y?.toFixed(0) }})
              </span>
            </div>
          </div>
        </div>
      </details>

      <!-- 航点列表面板（可折叠） -->
      <details class="data-card rounded-xl p-4 sm:p-6 neon-border group" open>
        <summary class="flex items-center justify-between cursor-pointer list-none [&::-webkit-details-marker]:hidden [&::marker]:hidden">
          <h3 class="text-lg font-bold flex items-center gap-2" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">
            <MapPin class="w-5 h-5 text-cyber-purple" />
            {{ t('map.waypointList') }}
          </h3>
          <div class="flex items-center gap-2">
            <span class="text-xs px-2 py-0.5 rounded-full bg-cyber-purple/20 text-cyber-purple">{{ waypoints.length }}</span>
            <svg class="w-4 h-4 transition-transform group-open:rotate-180" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round">
              <polyline points="6 9 12 15 18 9"></polyline>
            </svg>
          </div>
        </summary>
        <div class="mt-4 space-y-2 max-h-48 overflow-y-auto">
          <div
            v-for="wp in waypoints"
            :key="wp.id"
            class="p-3 rounded-lg border cursor-pointer transition-colors flex items-center gap-3"
            :class="selectedWaypoint?.id === wp.id
              ? 'border-cyber-purple bg-cyber-purple/10'
              : (theme === 'dark' ? 'border-industrial-600 hover:border-cyber-purple bg-industrial-700/30' : 'border-gray-200 hover:border-cyber-purple bg-gray-50')"
            @click="selectWaypoint(wp)"
          >
            <div
              class="w-8 h-8 rounded-full flex items-center justify-center shrink-0"
              :class="getWaypointBgClass(wp)"
            >
              <component :is="getWaypointIcon(wp)" class="w-4 h-4 text-white" />
            </div>
            <div class="flex-1 min-w-0">
              <p class="font-medium truncate" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ wp.name || wp.id }}</p>
              <p class="text-xs font-mono" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">({{ wp.x?.toFixed(0) }}, {{ wp.y?.toFixed(0) }})</p>
            </div>
          </div>
        </div>
      </details>
    </div>

    <!-- 创建任务弹窗 -->
    <div v-if="showTaskModal" class="fixed inset-0 bg-black/60 backdrop-blur-sm z-50 flex items-center justify-center p-4" @click.self="showTaskModal = false">
      <div class="data-card rounded-xl p-6 neon-border w-full max-w-md animate-scale-in">
        <div class="flex items-center justify-between mb-6">
          <h2 class="text-xl font-bold" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ t('map.dispatchRobot') }}</h2>
          <button class="p-2 rounded-lg transition-colors" :class="theme === 'dark' ? 'hover:bg-industrial-700 text-gray-400' : 'hover:bg-gray-100 text-gray-500'" @click="showTaskModal = false">
            <X class="w-5 h-5" />
          </button>
        </div>

        <!-- 目标航点信息 -->
        <div class="mb-5 p-3 rounded-lg border" :class="theme === 'dark' ? 'bg-cyber-purple/10 border-cyber-purple/30' : 'bg-purple-50 border-purple-200'">
          <p class="text-xs mb-1" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">{{ t('map.targetWaypoint') }}</p>
          <p class="text-base font-semibold text-cyber-purple">{{ selectedWaypoint?.name || selectedWaypoint?.id }}</p>
          <p v-if="selectedWaypoint" class="text-xs font-mono mt-1" :class="theme === 'dark' ? 'text-gray-500' : 'text-gray-400'">
            ({{ selectedWaypoint.x?.toFixed(1) }}, {{ selectedWaypoint.y?.toFixed(1) }})
          </p>
        </div>

        <!-- 可选机器人列表 -->
        <label class="block text-sm mb-3 font-medium" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('tasks.selectRobot') }}</label>
        <div class="space-y-3 max-h-64 overflow-y-auto mb-5">
          <div
            v-for="robot in availableRobots"
            :key="robot.id"
            class="p-4 rounded-xl border-2 cursor-pointer transition-all"
            :class="taskRobot === robot.id
              ? 'border-cyber-blue bg-cyber-blue/10 ring-2 ring-cyber-blue/20'
              : (theme === 'dark' ? 'border-industrial-600 bg-industrial-700/50 hover:border-industrial-500' : 'border-gray-200 bg-gray-50 hover:border-gray-300')"
            @click="taskRobot = robot.id"
          >
            <div class="flex items-center justify-between">
              <div class="flex items-center gap-3">
                <div class="flex items-center justify-center w-10 h-10 rounded-full" :class="taskRobot === robot.id ? 'bg-cyber-blue/20' : (theme === 'dark' ? 'bg-industrial-600' : 'bg-gray-200')">
                  <Bot class="w-5 h-5" :class="taskRobot === robot.id ? 'text-cyber-blue' : (theme === 'dark' ? 'text-gray-400' : 'text-gray-500')" />
                </div>
                <div>
                  <span class="font-medium" :class="theme === 'dark' ? 'text-white' : 'text-gray-900'">{{ robot.id }}</span>
                  <div class="flex items-center gap-2 mt-1 text-xs" :class="theme === 'dark' ? 'text-gray-400' : 'text-gray-500'">
                    <span class="flex items-center gap-1">
                      <Battery class="w-3 h-3" :class="getBatteryIconClass(robot.battery)" />
                      {{ robot.battery }}%
                    </span>
                    <span>{{ statusText(robot.status) }}</span>
                  </div>
                </div>
              </div>
              <!-- 选中标记 -->
              <div v-if="taskRobot === robot.id" class="w-6 h-6 rounded-full bg-cyber-blue flex items-center justify-center">
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

        <!-- 任务类型选择 -->
        <div class="mb-4">
          <label class="block text-sm mb-2 font-medium" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('tasks.taskType') }}</label>
          <div class="grid grid-cols-2 gap-2">
            <button
              v-for="tt in taskTypeOptions"
              :key="tt.value"
              class="py-3 rounded-lg text-sm font-medium transition-all"
              :class="taskType === tt.value
                ? 'bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue'
                : (theme === 'dark' ? 'bg-industrial-700 border border-industrial-600 text-gray-400' : 'bg-gray-100 border border-gray-200 text-gray-500')"
              @click="taskType = tt.value; if (tt.value === 1) taskSiteCode = 0"
            >
              {{ tt.label }}
            </button>
          </div>
        </div>

        <!-- 站点编码输入（巡航类型时隐藏） -->
        <div v-if="taskType !== 1" class="mb-4">
          <label class="block text-sm mb-2 font-medium" :class="theme === 'dark' ? 'text-gray-300' : 'text-gray-700'">{{ t('tasks.siteCode') }}</label>
          <input
            v-model.number="taskSiteCode"
            type="number"
            min="0"
            :placeholder="t('tasks.siteCodePlaceholder')"
            class="w-full px-4 py-2.5 border rounded-lg focus:border-cyber-blue focus:outline-none"
            :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-white' : 'bg-white border-gray-200 text-gray-900'"
          />
        </div>

        <!-- 操作按钮 -->
        <div class="flex gap-3 pt-2">
          <button
            class="flex-1 px-5 py-3 rounded-xl border transition-colors font-medium"
            :class="theme === 'dark' ? 'bg-industrial-700 border-industrial-600 text-gray-300 hover:bg-industrial-600' : 'bg-gray-100 border-gray-200 text-gray-600 hover:bg-gray-200'"
            @click="showTaskModal = false"
          >
            {{ t('common.cancel') }}
          </button>
          <button
            class="flex-1 px-5 py-3 rounded-xl bg-cyber-blue/20 border border-cyber-blue/40 text-cyber-blue hover:bg-cyber-blue/30 transition-colors font-medium"
            @click="confirmTask"
          >
            {{ t('map.confirmDispatch') }}
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup>
import { ref, computed, onMounted, onUnmounted, watch } from 'vue'
import { storeToRefs } from 'pinia'
import { useFleetStore } from '../stores/fleet'
import { useSettingsStore } from '../stores/settings'
import axios from 'axios'
import {
  Bot, MapPin, Battery, X, FolderOpen, RotateCcw, Plus, Minus,
  Zap, Car, RefreshCw
} from 'lucide-vue-next'

const fleetStore = useFleetStore()
const settingsStore = useSettingsStore()
const { language, theme } = storeToRefs(settingsStore)
const { t } = settingsStore

// DOM 引用与响应式状态
const mapCanvas = ref(null)
const mapWrapper = ref(null)
const selectedRobot = ref(null)
const selectedWaypoint = ref(null)
const showTaskModal = ref(false)
const taskRobot = ref('')
const taskType = ref(1)
const taskSiteCode = ref(0)

// 地图变换参数
const scale = ref(0.8)
const offset = ref({ x: 50, y: 50 })
const isDragging = ref(false)
const dragStart = ref({ x: 0, y: 0 })
const lastTouchDist = ref(0)
const lastTouchCenter = ref({ x: 0, y: 0 })

// 地图图片加载状态
const mapImage = ref(null)
const mapLoading = ref(false)

const waypoints = computed(() => fleetStore.waypoints || [])
const mapData = computed(() => fleetStore.mapData)

// 机器人列表（由 store 中对象转为数组）
const robotList = computed(() => {
  return Object.entries(fleetStore.robots).map(([id, data]) => ({ id, ...data }))
})

// 地图上显示的机器人标记（筛选有位置的机器人，包括 offline 使用最后已知位姿）
const robotMarkers = computed(() => {
  // 显示所有有位姿的机器人（包括 offline，用最后已知位姿）
  return robotList.value.filter(r => r.position)
})

// 链式撤退状态检测：检查是否有机器人处于链式撤退或避让任务中
const chainActive = computed(() => {
  return robotList.value.some(r =>
    r.current_task && (String(r.current_task).startsWith('chain_retreat_') || String(r.current_task).startsWith('avoidance_'))
  )
})

// 可用于派遣任务的机器人（在线且空闲）
const availableRobots = computed(() => {
  return robotList.value.filter(r => r.online && (r.status === 'idle' || r.status === 'arrived' || r.status === 'unknown'))
})

// 任务类型选项
const taskTypeOptions = computed(() => [
  { value: 1, label: t('tasks.taskTypeCRUISE') },
  { value: 2, label: t('tasks.taskTypeLOAD') },
  { value: 3, label: t('tasks.taskTypeUNLOAD') },
  { value: 4, label: t('tasks.taskTypeSITE_SPECIFIC') }
])

/** 地图坐标锚在圆心：固定 32x32，translate 只针对圆点，标签另 absolute 不参与尺寸 */
function getWaypointAnchorStyle(wp) {
  const x = wp.x * scale.value + offset.value.x
  const y = wp.y * scale.value + offset.value.y
  return {
    left: `${x}px`,
    top: `${y}px`,
    width: '32px',
    height: '32px',
    transform: 'translate(-50%, -50%)',
  }
}

/** 同上，机器人圆 40x40 (w-10) */
function getRobotAnchorStyle(robot) {
  if (!robot.position) return { display: 'none' }
  const x = robot.position.x * scale.value + offset.value.x
  const y = robot.position.y * scale.value + offset.value.y
  return {
    left: `${x}px`,
    top: `${y}px`,
    width: '40px',
    height: '40px',
    transform: 'translate(-50%, -50%)',
  }
}

/** 根据航点类型返回对应图标组件 */
function getWaypointIcon(wp) {
  if (wp.is_charging_station) return Zap
  if (wp.is_parking_spot) return Car
  return MapPin
}

/** 根据航点类型返回背景样式类 */
function getWaypointBgClass(wp) {
  if (wp.is_charging_station) return 'bg-cyber-green shadow-lg shadow-cyber-green/50'
  if (wp.is_parking_spot) return 'bg-cyber-orange shadow-lg shadow-cyber-orange/50'
  return 'bg-cyber-purple shadow-lg shadow-cyber-purple/50'
}

/** 获取航点类型名称 */
function getWaypointTypeName(wp) {
  if (wp.is_charging_station) return t('map.chargingStation')
  if (wp.is_parking_spot) return t('map.parkingSpot')
  return t('map.normalWaypoint')
}

/** 获取航点类型样式类 */
function waypointTypeClass(wp) {
  if (wp.is_charging_station) return 'bg-cyber-green/20 text-cyber-green'
  if (wp.is_parking_spot) return 'bg-cyber-orange/20 text-cyber-orange'
  return 'bg-cyber-purple/20 text-cyber-purple'
}

/** 获取机器人朝向旋转样式 */
function getRobotHeadingStyle(robot) {
  const deg = getRobotHeadingDeg(robot)
  return {
    transform: `rotate(${deg}deg)`,
    transformOrigin: '50% 50%',
  }
}

/** 获取机器人朝向箭头颜色 */
function getRobotArrowClass(robot) {
  if (robot.status === 'offline' || !robot.online) return 'border-b-slate-300'
  if (robot.status === 'moving' || robot.status === 'working') return 'border-b-cyan-300'
  if (robot.status === 'idle' || robot.status === 'arrived') return 'border-b-emerald-300'
  if (robot.status === 'failed') return 'border-b-rose-300'
  return 'border-b-violet-300'
}

/** 获取机器人朝向箭头发光效果 */
function getRobotArrowGlowClass(robot) {
  if (robot.status === 'offline' || !robot.online) return 'drop-shadow-[0_0_6px_rgba(203,213,225,0.85)]'
  if (robot.status === 'moving' || robot.status === 'working') return 'drop-shadow-[0_0_8px_rgba(103,232,249,0.9)]'
  if (robot.status === 'idle' || robot.status === 'arrived') return 'drop-shadow-[0_0_8px_rgba(110,231,183,0.9)]'
  if (robot.status === 'failed') return 'drop-shadow-[0_0_8px_rgba(253,164,175,0.9)]'
  return 'drop-shadow-[0_0_8px_rgba(196,181,253,0.9)]'
}

/** 计算机器人朝向角度（度），优先级：TF 偏航角 > 当前路段方向 > 规划路径方向 */
function getRobotHeadingDeg(robot) {
  // Preferred source: TF pose quaternion projected to yaw (already provided by backend as position.yaw).
  const yaw = robot?.position?.yaw
  if (Number.isFinite(yaw)) {
    // World yaw: +Y upward; screen/map pixel Y downward. Arrow default points upward.
    return (-yaw * 180) / Math.PI + 90
  }

  // Prefer live segment direction (most informative).
  if (robot.location_type === 'segment' && robot.current_segment) {
    const parts = String(robot.current_segment).split('->')
    if (parts.length === 2) {
      const a = parts[0].trim()
      const b = parts[1].trim()
      const pa = waypoints.value.find(w => w.id === a)
      const pb = waypoints.value.find(w => w.id === b)
      if (pa && pb) {
        const dx = (pb.x ?? 0) - (pa.x ?? 0)
        const dy = (pb.y ?? 0) - (pa.y ?? 0)
        if (Math.abs(dx) + Math.abs(dy) > 1e-6) return (Math.atan2(dy, dx) * 180) / Math.PI + 90
      }
    }
  }

  // Fallback: use planned_route progress if available.
  if (Array.isArray(robot.planned_route) && robot.planned_route.length >= 2) {
    const route = robot.planned_route
    const idx = getRouteProgressSegmentIndex(robot, route)
    const fromId = route[Math.max(0, Math.min(idx, route.length - 2))]
    const toId = route[Math.max(1, Math.min(idx + 1, route.length - 1))]
    const pa = waypoints.value.find(w => w.id === fromId)
    const pb = waypoints.value.find(w => w.id === toId)
    if (pa && pb) {
      const dx = (pb.x ?? 0) - (pa.x ?? 0)
      const dy = (pb.y ?? 0) - (pa.y ?? 0)
      if (Math.abs(dx) + Math.abs(dy) > 1e-6) return (Math.atan2(dy, dx) * 180) / Math.PI + 90
    }
  }

  // No heading information; keep a stable default.
  return 0
}

// 电量相关样式辅助函数
function getBatteryIconClass(battery) {
  if (battery > 60) return 'text-cyber-green'
  if (battery > 20) return 'text-cyber-orange'
  return 'text-cyber-red'
}

function getBatteryTextClass(battery) {
  if (battery > 60) return 'text-cyber-green'
  if (battery > 20) return 'text-cyber-orange'
  return 'text-cyber-red'
}

function batteryBarClass(battery) {
  if (battery > 60) return 'bg-cyber-green'
  if (battery > 20) return 'bg-cyber-orange'
  return 'bg-cyber-red'
}

/** 机器人状态文本映射 */
function statusText(status) {
  const map = {
    online: t('common.online'),
    offline: t('common.offline'),
    idle: t('common.idle'),
    moving: t('common.moving'),
    arrived: t('common.arrived'),
    failed: t('tasks.failed'),
    unknown: t('common.unknown')
  }
  return map[status] || status
}

/** 机器人状态样式类 */
function robotStatusClass(status) {
  const classes = {
    online: 'bg-cyber-green/20 text-cyber-green',
    idle: 'bg-cyber-green/20 text-cyber-green',
    working: 'bg-cyber-blue/20 text-cyber-blue',
    moving: 'bg-cyber-blue/20 text-cyber-blue',
    arrived: 'bg-cyber-purple/20 text-cyber-purple',
    offline: 'bg-cyber-red/20 text-cyber-red',
    failed: 'bg-cyber-red/20 text-cyber-red'
  }
  return classes[status] || 'bg-gray-500/20 text-gray-400'
}

/** 机器人状态圆点样式类 */
function robotStatusDotClass(status) {
  const classes = {
    online: 'bg-cyber-green',
    idle: 'bg-cyber-green',
    working: 'bg-cyber-blue',
    moving: 'bg-cyber-blue',
    arrived: 'bg-cyber-purple',
    offline: 'bg-cyber-red',
    failed: 'bg-cyber-red'
  }
  return classes[status] || 'bg-gray-500'
}

// 选中标记实体
function selectRobot(robot) {
  selectedRobot.value = robot
  selectedWaypoint.value = null
}

function selectWaypoint(wp) {
  selectedWaypoint.value = wp
  selectedRobot.value = null
}

// 缩放控制
function zoomIn() {
  scale.value = Math.min(2, scale.value + 0.1)
  drawMap()
}

function zoomOut() {
  scale.value = Math.max(0.3, scale.value - 0.1)
  drawMap()
}

// 鼠标滚轮缩放
function handleWheel(event) {
  event.preventDefault()
  const delta = event.deltaY > 0 ? -0.1 : 0.1
  scale.value = Math.max(0.3, Math.min(2, scale.value + delta))
  drawMap()
}

// 鼠标拖拽平移
function startDrag(event) {
  isDragging.value = true
  dragStart.value = { x: event.clientX - offset.value.x, y: event.clientY - offset.value.y }
}

function doDrag(event) {
  if (!isDragging.value) return
  offset.value = { x: event.clientX - dragStart.value.x, y: event.clientY - dragStart.value.y }
  drawMap()
}

function endDrag() {
  isDragging.value = false
}

// 触屏交互：单指拖拽、双指缩放/平移
function onTouchStart(event) {
  if (event.touches.length === 1) {
    isDragging.value = true
    dragStart.value = {
      x: event.touches[0].clientX - offset.value.x,
      y: event.touches[0].clientY - offset.value.y
    }
  } else if (event.touches.length === 2) {
    isDragging.value = false
    lastTouchDist.value = getTouchDist(event.touches)
    lastTouchCenter.value = getTouchCenter(event.touches)
  }
}

function onTouchMove(event) {
  if (event.touches.length === 1 && isDragging.value) {
    offset.value = {
      x: event.touches[0].clientX - dragStart.value.x,
      y: event.touches[0].clientY - dragStart.value.y
    }
    drawMap()
  } else if (event.touches.length === 2) {
    const dist = getTouchDist(event.touches)
    const center = getTouchCenter(event.touches)
    // 双指缩放
    const delta = (dist - lastTouchDist.value) * 0.005
    scale.value = Math.max(0.3, Math.min(2, scale.value + delta))
    // 双指平移
    offset.value.x += center.x - lastTouchCenter.value.x
    offset.value.y += center.y - lastTouchCenter.value.y
    lastTouchDist.value = dist
    lastTouchCenter.value = center
    drawMap()
  }
}

function onTouchEnd(event) {
  if (event.touches.length < 2) {
    isDragging.value = false
  }
  if (event.touches.length === 1) {
    isDragging.value = true
    dragStart.value = {
      x: event.touches[0].clientX - offset.value.x,
      y: event.touches[0].clientY - offset.value.y
    }
  }
}

// 触屏辅助函数
function getTouchDist(touches) {
  const dx = touches[0].clientX - touches[1].clientX
  const dy = touches[0].clientY - touches[1].clientY
  return Math.sqrt(dx * dx + dy * dy)
}

function getTouchCenter(touches) {
  return {
    x: (touches[0].clientX + touches[1].clientX) / 2,
    y: (touches[0].clientY + touches[1].clientY) / 2
  }
}

/** 点击地图空白区域取消选中 */
function handleMapClick(event) {
  selectedRobot.value = null
  selectedWaypoint.value = null
}

/** 绘制地图主流程 */
function drawMap() {
  const canvas = mapCanvas.value
  if (!canvas) return

  const wrapper = mapWrapper.value
  canvas.width = wrapper.clientWidth
  canvas.height = wrapper.clientHeight

  const ctx = canvas.getContext('2d')

  ctx.fillStyle = theme === 'dark' ? '#0d1117' : '#f3f4f6'
  ctx.fillRect(0, 0, canvas.width, canvas.height)

  ctx.save()
  ctx.translate(offset.value.x, offset.value.y)
  ctx.scale(scale.value, scale.value)

  if (mapImage.value) {
    ctx.drawImage(mapImage.value, 0, 0)
  }

  drawRoutes(ctx)
  drawRobotRoutes(ctx)

  ctx.restore()
}

/** 绘制航点之间的连接路径 */
function drawRoutes(ctx) {
  const wps = waypoints.value
  if (!wps || wps.length === 0) return

  const wpMap = {}
  wps.forEach(wp => {
    wpMap[wp.id] = wp
  })

  ctx.strokeStyle = theme === 'dark' ? 'rgba(59, 130, 246, 0.5)' : 'rgba(59, 130, 246, 0.7)'
  ctx.lineWidth = 2 / scale.value

  const drawnConnections = new Set()

  wps.forEach(wp => {
    if (!wp.connections || wp.connections.length === 0) return

    wp.connections.forEach(targetId => {
      const connectionKey = [wp.id, targetId].sort().join('-')
      if (drawnConnections.has(connectionKey)) return
      drawnConnections.add(connectionKey)

      const targetWp = wpMap[targetId]
      if (!targetWp) return

      ctx.beginPath()
      ctx.moveTo(wp.x, wp.y)
      ctx.lineTo(targetWp.x, targetWp.y)
      ctx.stroke()
    })
  })
}

/** 计算机器人在规划路径中的当前进度段索引 */
function getRouteProgressSegmentIndex(robot, route) {
  if (!route || route.length < 2) return 0

  if (robot.location_type === 'waypoint' && robot.current_waypoint) {
    const idx = route.indexOf(robot.current_waypoint)
    if (idx >= 0) return idx
  }

  if (robot.location_type === 'segment' && robot.current_segment) {
    const parts = robot.current_segment.split('->')
    if (parts.length === 2) {
      const from = parts[0].trim()
      const to = parts[1].trim()
      for (let i = 0; i < route.length - 1; i++) {
        if ((route[i] === from && route[i + 1] === to) || (route[i] === to && route[i + 1] === from)) {
          return i
        }
      }
    }
  }

  return 0
}

/** 绘制各机器人的规划路径 */
function drawRobotRoutes(ctx) {
  const wps = waypoints.value
  if (!wps || wps.length === 0) return

  const wpMap = {}
  wps.forEach(wp => {
    wpMap[wp.id] = wp
  })

  const doneColor = theme.value === 'dark' ? 'rgba(59, 130, 246, 0.35)' : 'rgba(59, 130, 246, 0.45)'
  const todoColor = 'rgba(249, 115, 22, 0.95)'

  for (const robot of Object.values(robotList.value)) {
    if (!robot.planned_route || robot.planned_route.length < 2) continue

    const route = robot.planned_route
    const progressSegIdx = getRouteProgressSegmentIndex(robot, route)

    ctx.lineWidth = 4 / scale.value
    ctx.lineCap = 'round'
    ctx.lineJoin = 'round'

    for (let i = 0; i < route.length - 1; i++) {
      const a = wpMap[route[i]]
      const b = wpMap[route[i + 1]]
      if (!a || !b) continue

      ctx.strokeStyle = i < progressSegIdx ? doneColor : todoColor
      ctx.beginPath()
      ctx.moveTo(a.x, a.y)
      ctx.lineTo(b.x, b.y)
      ctx.stroke()
    }
  }
}

function loadMapFile() {
  loadMapImage()
}

/** 从后端加载地图图片 */
async function loadMapImage() {
  if (mapLoading.value) return
  mapLoading.value = true

  try {
    const response = await axios.get('/api/map/image')
    if (response.data && response.data.image) {
      const img = new Image()
      img.onload = () => {
        mapImage.value = img
        drawMap()
        mapLoading.value = false
      }
      img.onerror = () => {
        console.error('Failed to load map image')
        mapLoading.value = false
      }
      img.src = response.data.image
    }
  } catch (error) {
    console.error('Failed to fetch map image:', error)
    mapLoading.value = false
  }
}

function refreshPositions() {
  loadMapImage()
}

/** 发送所选机器人到航点 */
async function sendToWaypoint() {
  if (!selectedRobot.value) return

  const availableWps = waypoints.value.filter(wp => !wp.is_charging_station)
  if (availableWps.length === 0) {
    fleetStore.addLog('warning', '没有可用的目标航点')
    return
  }

  selectedWaypoint.value = availableWps[0]
  showTaskModal.value = true
}

/** 发送所选机器人回充电站 */
async function returnToCharge() {
  if (!selectedRobot.value) return

  const chargingWps = waypoints.value.filter(wp => wp.is_charging_station)
  if (chargingWps.length === 0) {
    fleetStore.addLog('warning', '没有可用的充电站')
    return
  }

  try {
    const response = await axios.post('/api/tasks', {
      waypoint_id: chargingWps[0].id,
      robot_id: selectedRobot.value.id,
      priority: 1,
      task_type: 1,
      site_code: 0
    })

    if (response.data && response.data.success) {
      fleetStore.addLog('success', `${selectedRobot.value.id} 返回充电站`)
    }
  } catch (error) {
    console.error('Failed to return to charge:', error)
    fleetStore.addLog('error', `返回充电失败: ${error.response?.data?.detail || error.message}`)
  }
}

/** 紧急停止所选机器人 */
async function stopRobot() {
  if (!selectedRobot.value) return

  try {
    await axios.post(`/api/robots/${selectedRobot.value.id}/stop`)
    fleetStore.addLog('warning', `${selectedRobot.value.id} 紧急停止`)
  } catch (error) {
    console.error('Failed to stop robot:', error)
    fleetStore.addLog('error', `停止失败: ${error.response?.data?.detail || error.message}`)
  }
}

function createTaskHere() {
  showTaskModal.value = true
}

/** 确认创建任务 */
async function confirmTask() {
  if (!taskRobot.value || !selectedWaypoint.value) {
    return
  }

  try {
    const response = await axios.post('/api/tasks', {
      waypoint_id: selectedWaypoint.value.id,
      robot_id: taskRobot.value,
      priority: 0,
      task_type: taskType.value,
      site_code: taskSiteCode.value
    })

    if (response.data && response.data.success) {
      fleetStore.addLog('success', `任务 ${response.data.task_id}: ${taskRobot.value} -> ${selectedWaypoint.value.id}`)
    }
  } catch (error) {
    console.error('Failed to create task:', error)
    fleetStore.addLog('error', `任务创建失败: ${error.response?.data?.detail || error.message}`)
  }

  showTaskModal.value = false
  taskRobot.value = ''
  taskType.value = 1
  taskSiteCode.value = 0
}

onMounted(() => {
  drawMap()
  loadMapImage()
})

// 监听地图数据变化、ROS 连接、航点、缩放、偏移等，触发重绘
watch(mapData, (newData) => {
  if (newData) {
    loadMapImage()
  }
})

watch(() => fleetStore.rosConnected, (connected) => {
  if (connected) {
    loadMapImage()
  }
})

watch(waypoints, () => {
  drawMap()
}, { deep: true })

watch(scale, () => {
  drawMap()
})

watch(offset, () => {
  drawMap()
}, { deep: true })
</script>
