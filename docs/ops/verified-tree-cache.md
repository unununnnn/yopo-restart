# 已验树缓存

`scripts/premerge.sh` 查找时以“当前 PR head 合入基线后”的 tree 为键，但登记的
key 始终是真正跑过回归的 tree。premerge 未命中时回归在 PR head worktree 中跑，
因此登记 `${sha}^{tree}`；`record-verified-tree.sh` 登记 `HEAD^{tree}`。只有待合入 tree
与某次真实测过的 tree 完全相同时才会命中。缓存整份 TSV 必须合法且唯一匹配，
才会跳过 `scripts/regress.sh`；trailer 检查和合入步骤仍照常执行。
默认文件是 `/workspace/yopo-ops/verified-trees.tsv`，测试或本地演练可设置
`PREMERGE_VERIFIED_TREES`。

推荐流程是分支先合入最新基线，再跑回归、登记和入队：

```bash
git fetch origin main
git merge origin/main
log=$(mktemp)
scripts/regress.sh 2>&1 | tee "$log"
out=$(sed -n 's/^pytest=.* 输出 //p' "$log" | tail -1)
cp "$log" "$out/regress.log"
scripts/record-verified-tree.sh "$out"
```

若 HEAD 未包含 `origin/main`，登记仍会保存已测的 HEAD tree，并打印提示；但
premerge 查找的合并后 tree 不同，所以该记录不会命中。

登记要求工作区完全干净，且 pytest、core、批跑均通过。当前 `regress.sh` 的输出没有
记录完整 HEAD sha，因此脚本只能校验 OUT 的更新时间不早于当前提交；这是现阶段的限制。

合入前基线仍会再次 fetch。若命中后基线前进，premerge 会比较追基线后的最终 tree；
tree 变化时会拒绝合入并要求重新跑全量回归，不会用旧记录放行新 tree。
