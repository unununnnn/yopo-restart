# 已验树缓存

`scripts/premerge.sh` 以“当前 PR head 合入基线后”的 tree 为键。缓存整份 TSV
必须合法且唯一匹配，才会跳过 `scripts/regress.sh`；trailer 检查和合入步骤仍照常执行。
默认文件是 `/workspace/yopo-ops/verified-trees.tsv`，测试或本地演练可设置
`PREMERGE_VERIFIED_TREES`。

作者跑回归时需把完整输出保存进回归生成的 OUT 目录，再登记：

```bash
log=$(mktemp)
scripts/regress.sh 2>&1 | tee "$log"
out=$(sed -n 's/^pytest=.* 输出 //p' "$log" | tail -1)
cp "$log" "$out/regress.log"
scripts/record-verified-tree.sh "$out"
```

登记要求工作区完全干净，且 pytest、core、批跑均通过。当前 `regress.sh` 的输出没有
记录完整 HEAD sha，因此脚本只能校验 OUT 的更新时间不早于当前提交；这是现阶段的限制。

合入前基线仍会再次 fetch。若命中后基线前进，premerge 会比较追基线后的最终 tree；
tree 变化时会拒绝合入并要求重新跑全量回归，不会用旧记录放行新 tree。
