# RTKLIB-EX 补丁目录(WP-a 填写)

- 文件名决定应用顺序(`list(SORT)`):`P1_maxprncmp.patch`、`P2_export.patch`、…
- 生成方式:把 `upstream/` 复制到临时目录改动后,在 `third_party/rtklib/` 下执行
  `diff -ruN upstream/src <tmp>/src | sed 's#^--- upstream/#--- a/#; s#^+++ <tmp>/#+++ b/#'`,
  即补丁路径形如 `a/src/rtklib.h` / `b/src/rtklib.h`,由 CMake 在构建树副本根目录以 `patch -p1` 应用。
- 上游原样目录 `upstream/` 永不改动;`sofa.c` 已在复制后删除,补丁不得引用它。
- 每个补丁在文件头写明目的、涉及行号与 DESIGN 节号;`VENDORED.md` 的补丁清单同步更新。
- 约定的补丁:
  - P1 `rtklib.h:211` `MAXPRNCMP` 加 `#ifndef` 并设 63;加 `MAXSAT<=255` 编译期断言(obsd_t.sat 为 uint8)。
  - P2 `pntpos.c` 末尾导出 `rtkx_prange/rtkx_varerr/rtkx_gettgd`,以及 estpos/estvel 的完整 x 与 Q。
