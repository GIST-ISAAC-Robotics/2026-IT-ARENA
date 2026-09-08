import {Workbook} from '@oai/artifact-tool';
const wb=Workbook.create();
console.log(wb.help('*',{search:'worksheet.*position|worksheet.*move|worksheets.*reorder|worksheet.*index',include:'index,examples,notes',maxChars:5000}).ndjson);
